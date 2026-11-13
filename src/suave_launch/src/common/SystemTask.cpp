//
// Created by suave on 4/18/24.
//

#include "SystemTask.h"

#include <chrono>
#include <csignal>
#include <cstring>
#include <thread>
#include <unistd.h>

std::string SystemTask::command_line() const
{
    std::string command{};
    for (size_t i = 0; i < m_commands.size(); ++i)
    {
        if (i > 0) command += " && ";
        command += m_commands[i];
    }
    if (!m_should_print_output) command += " > /dev/null 2>&1";
    return command;
}

TaskResult SystemTask::start()
{
    // Runs in the forked child only. After fork() in a multithreaded program only
    // async-signal-safe calls are allowed until exec, so no logging or allocation here
    // beyond what was prepared by the parent.
    execl("/bin/bash", "bash", "-c", m_prepared_command.c_str(), static_cast<char*>(nullptr));
    return TaskResult::FAILURE;
}

namespace
{
// Waits up to timeout for the child to exit; true if it did.
bool wait_exit(pid_t pid, int& status, std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        const pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid || r == -1) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}
}

void SystemTask::stop()
{
    if (m_pid <= 0) return;

    suave_log << "Stopping process group: " << m_pid << std::endl;
    int status = 0;

    // SIGINT first (lets ros2 launch / ros2 bag shut down cleanly), then SIGTERM, then
    // SIGKILL. Never block forever: a stuck child must not stall the caller.
    const std::pair<int, std::chrono::milliseconds> steps[] = {
        { SIGINT, std::chrono::milliseconds(5000) },
        { SIGTERM, std::chrono::milliseconds(3000) },
        { SIGKILL, std::chrono::milliseconds(2000) },
    };
    bool exited = false;
    for (const auto& [sig, timeout] : steps)
    {
        if (kill(-m_pid, sig) == -1 && errno == ESRCH)
        {
            exited = wait_exit(m_pid, status, std::chrono::milliseconds(100));
            break;
        }
        if (wait_exit(m_pid, status, timeout))
        {
            exited = true;
            break;
        }
        suave_warn << "Process group " << m_pid << " still running after signal " << sig << std::endl;
    }
    // Clean up anything left in the group (e.g. a child that outlived bash).
    kill(-m_pid, SIGKILL);

    if (exited) suave_log << "Process group " << m_pid << " stopped" << std::endl;
    else suave_err << "Process group " << m_pid << " did not exit" << std::endl;
    m_pid = -1;
}

std::future<TaskResult>* SystemTask::start_in_thread()
{
    if (m_pid > 0)
    {
        suave_err << "Process already running" << std::endl;
        return nullptr;
    }

    m_prepared_command = command_line();
    suave_log << "Executing command: " << m_prepared_command << std::endl;

    const pid_t pid = fork();
    if (pid == 0)
    {
        // Child: own process group so stop() can signal every descendant, and default
        // signal dispositions (the parent's handlers must never run in here).
        setpgid(0, 0);
        signal(SIGINT, SIG_DFL);
        signal(SIGTERM, SIG_DFL);
        signal(SIGQUIT, SIG_DFL);
        this->start();
        _exit(127);  // exec failed; _exit skips the parent's atexit handlers and destructors
    }
    if (pid < 0)
    {
        suave_err << "Failed to fork process" << std::endl;
        return nullptr;
    }

    setpgid(pid, pid);  // same as the child does; whichever runs first wins the race
    m_pid = pid;
    return nullptr;
}

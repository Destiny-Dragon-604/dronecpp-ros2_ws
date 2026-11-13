//
// Created by suave on 4/18/24.
//

#ifndef SYSTEMTASK_H
#define SYSTEMTASK_H

#include <string>
#include <sys/wait.h>
#include <utility>
#include <vector>

#include "ITask.h"

class SystemTask : public ITask
{
public:
    explicit SystemTask(std::vector<std::string> commands, bool should_print_output = false) :
        m_commands(std::move(commands)),
        m_should_print_output(should_print_output)
    {
    }

    TaskResult start() override;

    void stop() override;

    // Forks and runs the commands (joined with &&) in bash, in their own process group.
    // Warning: This function will always return nullptr
    std::future<TaskResult>* start_in_thread() override;

    std::string command_line() const;

private:
    pid_t m_pid = -1;
    std::string m_prepared_command{};
    std::vector<std::string> m_commands;
    bool m_should_print_output;
};

#endif //SYSTEMTASK_H

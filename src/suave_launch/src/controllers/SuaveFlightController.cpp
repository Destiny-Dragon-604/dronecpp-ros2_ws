#include "SuaveFlightController.h"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../common/SystemTask.h"
#include "../mavutil/MavUtil.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/VIOBridge.h"
#include "ControllerMacros.h"

namespace
{
constexpr float DEFAULT_TAKEOFF_ALT_M = 1.75f;
constexpr float MAX_ALT_M = 5.0f;   // takeoff altitude limit
constexpr float MAX_STEP_M = 5.0f;  // largest single move/goto distance accepted
}

SuaveFlightController::SuaveFlightController() :
    ISuaveController()
{
    m_sitl = is_sitl_connection();
    // Throws if the connection cannot be opened; caught in main().
    m_drone = std::make_unique<Drone>(connectToPX4(m_mavsdk));
    m_logger = std::make_unique<FlightLogger>(*m_drone, m_sitl);
}

void SuaveFlightController::start()
{
    if (m_sitl)
    {
        suave_log << "SITL connection: skipping RealSense / RTAB-Map / VIO" << std::endl;
    }
    else
    {
        start_vio_stack();
    }

    print_help();
    command_loop();
    shutdown();
}

void SuaveFlightController::start_vio_stack()
{
    const auto slam_launch = ament_index_cpp::get_package_share_directory("suave_launch") + "/launch/suave_slam.py";

    auto realsense_task = std::make_shared<SystemTask>(
        std::vector<std::string>{
            "source /opt/ros/humble/setup.bash",
            "ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true"
        }
    );
    auto rtabmap_task = std::make_shared<SystemTask>(
        std::vector<std::string>{
            "source /opt/ros/humble/setup.bash",
            "ros2 param set /camera/camera depth_module.emitter_enabled 0",
            "ros2 launch " + slam_launch
        }
    );
    auto vio_spinner = std::make_shared<RosNodeSpinner>();
    auto vio_bridge = std::make_shared<VIOBridge>(m_drone->system(), m_drone->initial_heading_rad());
    vio_bridge->set_vision_listener([this](const Mocap::Odometry& o) { m_logger->vision(o); });
    vio_spinner->add_node(vio_bridge);

    m_tasks = { realsense_task, rtabmap_task, vio_spinner };

    suave_log << "Starting RealSense" << std::endl;
    realsense_task->start_in_thread();
    sleep(3)

    suave_log << "Starting RTAB-Map" << std::endl;
    rtabmap_task->start_in_thread();
    sleep(5)

    suave_log << "Move the drone around by hand to build the initial map, then press enter" << std::endl;
    std::cin.clear();
    await_confirmation;

    suave_log << "Starting VIO bridge (RTAB-Map /odom -> PX4 external vision)" << std::endl;
    vio_spinner->start_in_thread();
}

bool SuaveFlightController::enter_offboard()
{
    if (m_offboard_active)
    {
        return true;
    }

    // Capture the takeoff point (position + yaw); goto targets are relative to it.
    m_drone->set_local_position_setpoint();

    const auto arm_result = m_drone->action().arm();
    if (arm_result != Action::Result::Success)
    {
        suave_err << "Arm failed: " << arm_result << std::endl;
        m_logger->event("error", "arm failed");
        return false;
    }
    // PX4 needs a setpoint streaming before it accepts offboard mode.
    if (!check("offboard setpoint", m_drone->offboard_setpoint()) ||
        !check("offboard start", m_drone->offboard().start()))
    {
        m_drone->action().disarm();
        return false;
    }
    m_offboard_active = true;
    return true;
}

void SuaveFlightController::command_loop()
{
    std::string line{};
    while (!m_end_controller)
    {
        std::cout << "> " << std::flush;
        if (!std::getline(std::cin, line))
        {
            break;  // EOF (Ctrl-D)
        }

        std::istringstream in{ line };
        std::string cmd{};
        if (!(in >> cmd))
        {
            continue;
        }
        m_logger->event("command", line);

        try
        {
            if (cmd == "help")
            {
                print_help();
            }
            else if (cmd == "takeoff")
            {
                float alt = DEFAULT_TAKEOFF_ALT_M;
                in >> alt;
                if (!(alt > 0.0f && alt <= MAX_ALT_M))
                {
                    suave_err << "takeoff altitude must be in (0, " << MAX_ALT_M << "] m" << std::endl;
                    m_logger->event("error", "takeoff altitude out of range");
                    continue;
                }
                if (enter_offboard())
                {
                    check("takeoff", m_drone->set_local_position_ned(0, 0, -alt));
                }
            }
            else if (cmd == "move" || cmd == "goto")
            {
                float n{}, e{}, d{};
                if (!(in >> n >> e >> d))
                {
                    suave_err << "usage: " << cmd << " <north> <east> <down>   (metres, down < 0 is up)" << std::endl;
                    continue;
                }
                if (!m_offboard_active)
                {
                    suave_err << "not in offboard; use takeoff first" << std::endl;
                    continue;
                }
                if (std::sqrt(n * n + e * e + d * d) > MAX_STEP_M)
                {
                    suave_err << "rejected: more than " << MAX_STEP_M << " m" << std::endl;
                    m_logger->event("error", cmd + " rejected: step larger than limit");
                    continue;
                }
                if (cmd == "move")
                {
                    check("move", m_drone->set_relative_position_ned(n, e, d));
                }
                else
                {
                    check("goto", m_drone->set_local_position_ned(n, e, d));
                }
            }
            else if (cmd == "hold")
            {
                if (m_offboard_active)
                {
                    check("hold", m_drone->offboard_hold());
                }
            }
            else if (cmd == "land")
            {
                if (m_offboard_active)
                {
                    check("land", m_drone->offboard_wait_for_land());
                    m_offboard_active = false;
                }
            }
            else if (cmd == "status")
            {
                suave_log << m_drone->get_ned_position_string() << std::endl;
                suave_log << "Heading: " << m_drone->heading_deg() << " deg, armed: " << m_drone->armed()
                          << ", in air: " << m_drone->in_air() << ", offboard: " << m_offboard_active << std::endl;
            }
            else if (cmd == "mark")
            {
                std::string note{};
                std::getline(in >> std::ws, note);
                m_logger->event("mark", note);
                suave_log << "marked" << std::endl;
            }
            else if (cmd == "log")
            {
                suave_log << "Logging to " << m_logger->dir().string() << std::endl;
            }
            else if (cmd == "wind" || cmd == "turb" || cmd == "gust" || cmd == "push" || cmd == "torque" ||
                     cmd == "drag" || cmd == "calm" || cmd == "dstatus")
            {
                send_disturbance(line);
            }
            else if (cmd == "exit" || cmd == "quit")
            {
                break;
            }
            else
            {
                suave_err << "unknown command '" << cmd << "' (type help)" << std::endl;
                m_logger->event("error", "unknown command");
            }
        }
        catch (const std::exception& ex)
        {
            // e.g. telemetry not received yet
            suave_err << cmd << " failed: " << ex.what() << std::endl;
            m_logger->event("error", cmd + " failed: " + ex.what());
        }
    }
}

void SuaveFlightController::print_help() const
{
    std::cout <<
        "Commands (NED frame, metres; down < 0 is up):\n"
        "  takeoff [alt]      arm, enter offboard, climb to alt above the takeoff point (default 1.75)\n"
        "  move dn de dd      step relative to the current position\n"
        "  goto n e d         fly to a point relative to the takeoff point\n"
        "  hold               hold the current position\n"
        "  land               descend at 0.5 m/s, disarm on touchdown\n"
        "  status             position, heading, armed / in-air / offboard state\n"
        "  exit               land if airborne, stop camera/VIO, quit\n"
        "Logging (always on):\n"
        "  mark <note>        add a note to events.csv at this instant\n"
        "  log                show this flight's data folder\n";
    if (m_sitl)
    {
        std::cout <<
        "Disturbances (SITL only; NED frame, see tools/disturb.py --help):\n"
        "  wind n e d         steady wind (m/s), e.g. wind 0 5 0 = 5 m/s blowing toward east\n"
        "  turb sigma [L]     add Dryden-style turbulence, sigma m/s, length scale L m (default 20)\n"
        "  gust n e d dur     1-cosine gust peaking at (n,e,d) m/s over dur s\n"
        "  push n e d dur     external force (N) for dur s\n"
        "  torque r p y dur   external torque about body roll/pitch/yaw axes (N m) for dur s\n"
        "  drag cd axy az     body drag model (default 1.0 0.05 0.12)\n"
        "  calm               remove all disturbances\n"
        "  dstatus            print the current disturbance state\n";
    }
    std::cout << "Ctrl-C at any time lands and exits." << std::endl;
}

bool SuaveFlightController::check(const std::string& what, const Offboard::Result result)
{
    std::ostringstream text;
    text << what << ": " << result;
    m_logger->event("result", text.str());
    if (result == Offboard::Result::Success)
    {
        suave_log << what << ": ok" << std::endl;
        return true;
    }
    suave_err << what << " failed: " << result << std::endl;
    return false;
}

void SuaveFlightController::send_disturbance(const std::string& line)
{
    if (!m_sitl)
    {
        suave_err << "disturbances only exist in simulation" << std::endl;
        return;
    }
    // The line goes into a shell command: allow only plain numbers and words.
    for (const char c : line)
    {
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == ' ' || c == '.' || c == '-' || c == '_'))
        {
            suave_err << "invalid character in disturbance command" << std::endl;
            return;
        }
    }
    const std::string cmd = (line.rfind("dstatus", 0) == 0) ? std::string("status") : line;
    const std::string shell =
        "GZ_IP=${GZ_IP:-127.0.0.1} gz topic -t /suave/disturbance -m gz.msgs.StringMsg -p 'data: \"" + cmd + "\"' > /dev/null 2>&1";
    if (cmd == "status")
    {
        // Show the plugin's reply (it answers on /suave/disturbance/status).
        const std::string echo = "GZ_IP=${GZ_IP:-127.0.0.1} timeout 3 gz topic -e -n 1 -t /suave/disturbance/status & sleep 0.3; " + shell + "; wait";
        std::system(echo.c_str());
    }
    else if (std::system(shell.c_str()) != 0)
    {
        suave_err << "could not reach the disturbance plugin (is the sim started with tools/sim.sh?)" << std::endl;
        m_logger->event("error", "disturbance command not delivered: " + line);
    }
}

void SuaveFlightController::shutdown()
{
    if (m_shutdown_done.exchange(true))
    {
        return;
    }
    m_end_controller = true;
    suave_log << "SuaveFlightController::shutdown()" << std::endl;

    if (m_drone)
    {
        try
        {
            if (m_offboard_active)
            {
                m_drone->offboard_wait_for_land();
            }
            else if (m_drone->in_air())
            {
                m_drone->action().land();
            }
        }
        catch (const std::exception& ex)
        {
            suave_err << "Landing during shutdown failed: " << ex.what() << std::endl;
        }
    }

    for (auto& task : m_tasks)
    {
        task->stop();
    }

    if (m_logger)
    {
        m_logger->finish();
    }
}

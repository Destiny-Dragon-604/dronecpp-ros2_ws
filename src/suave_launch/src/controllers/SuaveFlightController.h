#ifndef SUAVEFLIGHTCONTROLLER_H
#define SUAVEFLIGHTCONTROLLER_H

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "../common/ISuaveController.h"
#include "../common/ITask.h"
#include "../logging/FlightLogger.h"
#include "../mavutil/Drone.h"

// Minimal offboard flight controller.
//
// Real drone: starts RealSense + RTAB-Map, bridges RTAB-Map /odom into PX4 as external
// vision, then takes console commands.
// SITL (SUAVE_MAVLINK_SERIAL_ID unset or "SITL"): skips the camera/VIO stack and flies on
// PX4's own state estimate.
class SuaveFlightController : public ISuaveController {
public:
    SuaveFlightController();
    void start() override;
    void shutdown() override;

private:
    void start_vio_stack();
    void command_loop();
    bool enter_offboard();
    void print_help() const;
    bool check(const std::string& what, Offboard::Result result);
    void send_disturbance(const std::string& line);

    bool m_sitl{ false };
    std::atomic<bool> m_end_controller{ false };
    std::atomic<bool> m_shutdown_done{ false };
    std::atomic<bool> m_offboard_active{ false };
    // Declared before m_drone so it is destroyed after it: MAVSDK callbacks that write to the
    // logger stop when the Drone (and its Telemetry) goes away.
    std::unique_ptr<FlightLogger> m_logger{ nullptr };
    std::unique_ptr<Drone> m_drone{ nullptr };
    std::vector<std::shared_ptr<ITask>> m_tasks{};
};

#endif //SUAVEFLIGHTCONTROLLER_H

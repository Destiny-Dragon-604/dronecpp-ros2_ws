#ifndef FLIGHTLOGGER_H
#define FLIGHTLOGGER_H

#include <atomic>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <plugins/mocap/mocap.h>

#include "../common/SystemTask.h"
#include "../mavutil/Drone.h"
#include "CsvWriter.h"

// Records everything about a flight into data/flight_<YYYYmmdd_HHMMSS>/:
//
//  * one CSV per MAVSDK telemetry stream (position/velocity, attitude, rates, IMU raw /
//    scaled / high-res, odometry, actuator controls + outputs, battery, GPS, health, modes,
//    status text, ...), state-like streams only when they change
//  * setpoints.csv  - every offboard setpoint sent to PX4
//  * events.csv     - console commands, results, marks, warnings
//  * vision_odometry.csv - VIO estimates forwarded to PX4 (real drone)
//  * rosbag/        - all ROS 2 topics (ros2 bag record -a)
//  * *.ulg          - the PX4 flight log (SITL: copied automatically at the end)
//  * disturbance.csv - written by the Gazebo disturbance plugin in SITL (wind, forces, truth)
//  * metadata.txt / summary.txt
//
// Data root: $SUAVE_DATA_DIR, else <workspace>/data.
class FlightLogger
{
public:
    FlightLogger(Drone& drone, bool sitl);
    ~FlightLogger();

    FlightLogger(const FlightLogger&) = delete;
    FlightLogger& operator=(const FlightLogger&) = delete;

    const std::filesystem::path& dir() const { return m_dir; }

    void event(const std::string& kind, const std::string& text);
    void vision(const mavsdk::Mocap::Odometry& odometry);

    // Stops the rosbag, copies the PX4 ULog (SITL) and writes summary.txt. Idempotent.
    void finish();

    static std::filesystem::path data_root();
    // Workspace root (<ws> in <ws>/install/suave_launch/...), empty if not found.
    static std::filesystem::path workspace_dir();

    // Call once at startup, before connecting: keeps the console to real warnings, errors
    // and PX4 status messages, and records every MAVSDK log line in mavsdk_log.csv of the
    // active flight.
    static void install_mavsdk_log_filter();

private:
    CsvWriter& csv(const std::string& name, const std::string& header);
    void subscribe_all();
    void set_rates();
    void write_metadata();
    void start_rosbag();
    void copy_px4_ulog();
    void write_summary();
    void export_flight();

    Drone& m_drone;
    bool m_sitl;
    std::filesystem::path m_dir{};
    std::filesystem::path m_marker{};
    CsvWriter::Clock::time_point m_origin{ CsvWriter::Clock::now() };
    std::filesystem::file_time_type m_start_file_time{};

    std::mutex m_writers_mutex{};
    std::map<std::string, std::unique_ptr<CsvWriter>> m_writers{};

    std::shared_ptr<SystemTask> m_rosbag{};
    std::thread m_setup_thread{};
    std::atomic<bool> m_finished{ false };

    // Last values of state-like streams, to log only on change.
    std::mutex m_state_mutex{};
    std::map<std::string, std::string> m_last_state{};
    bool changed(const std::string& stream, const std::string& value);

    void mavsdk_log(int level, const std::string& message, const std::string& location);
};

#endif //FLIGHTLOGGER_H

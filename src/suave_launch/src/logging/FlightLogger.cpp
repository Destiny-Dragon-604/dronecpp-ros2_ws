#include "FlightLogger.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <log_callback.h>
#include <plugins/info/info.h>

#include "../util/Util.h"

namespace fs = std::filesystem;

namespace
{
std::string timestamp_string()
{
    const auto now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

struct PipeCloser
{
    void operator()(FILE* f) const { if (f) pclose(f); }
};

std::string run(const std::string& cmd)
{
    std::array<char, 256> buf{};
    std::string out{};
    std::unique_ptr<FILE, PipeCloser> pipe(popen(cmd.c_str(), "r"));
    if (!pipe) return out;
    while (fgets(buf.data(), buf.size(), pipe.get())) out += buf.data();
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
    return out;
}

template<typename T>
std::string to_str(const T& v)
{
    std::ostringstream ss;
    ss << v;
    return ss.str();
}
}

namespace
{
// The flight currently receiving MAVSDK log lines (guarded so finish() can detach safely).
std::mutex s_active_mutex;
FlightLogger* s_active{ nullptr };

// MAVSDK lines that are expected and harmless, kept out of the console (still logged).
bool is_noise(const std::string& m)
{
    static const char* patterns[] = {
        "User callback queue too slow", "See: https://mavsdk", "System status is usually fixed",
        "Dropping command", "Received ack for not-existing command", "Vehicle type changed",
        "sending again", "Retrying failed",
    };
    for (const char* p : patterns)
        if (m.find(p) != std::string::npos) return true;
    return false;
}
}

void FlightLogger::install_mavsdk_log_filter()
{
    mavsdk::log::subscribe([](mavsdk::log::Level level, const std::string& message,
                              const std::string& file, int line) {
        {
            std::lock_guard<std::mutex> lock{ s_active_mutex };
            if (s_active) s_active->mavsdk_log(static_cast<int>(level), message, file + ":" + std::to_string(line));
        }
        if (message.rfind("MAVLink: ", 0) == 0)
        {
            // PX4 status text ("Armed by external command", "Takeoff detected", failsafes, ...)
            std::cout << "PX4: " << message.substr(9) << std::endl;
        }
        else if (level == mavsdk::log::Level::Err || (level == mavsdk::log::Level::Warn && !is_noise(message)))
        {
            std::cerr << "MAVSDK " << (level == mavsdk::log::Level::Err ? "error" : "warning") << ": " << message << std::endl;
        }
        return true;  // we printed what matters; suppress MAVSDK's own output
    });
}

void FlightLogger::mavsdk_log(int level, const std::string& message, const std::string& location)
{
    static const char* names[] = { "debug", "info", "warn", "error" };
    auto& w = csv("mavsdk_log", "level,message,location");
    w.write(w.begin() << names[std::clamp(level, 0, 3)] << message << location);
}

fs::path FlightLogger::data_root()
{
    if (const char* env = std::getenv("SUAVE_DATA_DIR"); env && *env)
    {
        return fs::path(env);
    }
    try
    {
        // <ws>/install/suave_launch/share/suave_launch -> <ws>
        const fs::path share{ ament_index_cpp::get_package_share_directory("suave_launch") };
        const auto ws = fs::weakly_canonical(share / ".." / ".." / ".." / "..");
        if (fs::exists(ws / "src"))
        {
            return ws / "data";
        }
    }
    catch (...) { /* fall through */ }
    const char* home = std::getenv("HOME");
    return fs::path(home ? home : ".") / "suave_data";
}

FlightLogger::FlightLogger(Drone& drone, const bool sitl) :
    m_drone(drone),
    m_sitl(sitl)
{
    const auto root = data_root();
    m_dir = root / ("flight_" + timestamp_string());
    fs::create_directories(m_dir);
    m_start_file_time = fs::file_time_type::clock::now();

    // Tells the Gazebo disturbance plugin where this flight's folder is.
    m_marker = root / ".active_flight";
    std::ofstream(m_marker, std::ofstream::trunc) << fs::absolute(m_dir).string() << '\n';

    suave_log << "Logging flight data to " << m_dir.string() << std::endl;
    {
        std::lock_guard<std::mutex> lock{ s_active_mutex };
        s_active = this;
    }

    event("start", sitl ? "SITL" : "hardware");
    subscribe_all();
    start_rosbag();

    // Rate requests and version queries wait for PX4 acks; keep them off the main thread.
    m_setup_thread = std::thread([this]() {
        set_rates();
        write_metadata();
    });
}

FlightLogger::~FlightLogger()
{
    finish();
}

CsvWriter& FlightLogger::csv(const std::string& name, const std::string& header)
{
    std::lock_guard<std::mutex> lock{ m_writers_mutex };
    auto& writer = m_writers[name];
    if (!writer)
    {
        writer = std::make_unique<CsvWriter>(m_dir / (name + ".csv"), header, m_origin);
    }
    return *writer;
}

bool FlightLogger::changed(const std::string& stream, const std::string& value)
{
    std::lock_guard<std::mutex> lock{ m_state_mutex };
    auto it = m_last_state.find(stream);
    if (it != m_last_state.end() && it->second == value) return false;
    m_last_state[stream] = value;
    return true;
}

void FlightLogger::event(const std::string& kind, const std::string& text)
{
    auto& w = csv("events", "kind,text");
    w.write(w.begin() << kind << text);
}

void FlightLogger::vision(const Mocap::Odometry& o)
{
    auto& w = csv("vision_odometry", "time_usec,x_m,y_m,z_m,qw,qx,qy,qz,vx_m_s,vy_m_s,vz_m_s,roll_rate_rad_s,pitch_rate_rad_s,yaw_rate_rad_s");
    w.write(w.begin() << o.time_usec
        << o.position_body.x_m << o.position_body.y_m << o.position_body.z_m
        << o.q.w << o.q.x << o.q.y << o.q.z
        << o.speed_body.x_m_s << o.speed_body.y_m_s << o.speed_body.z_m_s
        << o.angular_velocity_body.roll_rad_s << o.angular_velocity_body.pitch_rad_s << o.angular_velocity_body.yaw_rad_s);
}

void FlightLogger::subscribe_all()
{
    using T = Telemetry;
    auto& tel = m_drone.telemetry();

    // --- Streams owned by Drone: attach as listeners --------------------------------------
    {
        auto& w = csv("position_velocity_ned", "n_m,e_m,d_m,vn_m_s,ve_m_s,vd_m_s");
        m_drone.position_velocity_ned_stream().add_listener([&w](T::PositionVelocityNed p) {
            w.write(w.begin() << p.position.north_m << p.position.east_m << p.position.down_m
                << p.velocity.north_m_s << p.velocity.east_m_s << p.velocity.down_m_s);
        });
    }
    {
        auto& w = csv("attitude_quaternion", "timestamp_us,qw,qx,qy,qz");
        m_drone.attitude_quaternion_stream().add_listener([&w](T::Quaternion q) {
            w.write(w.begin() << q.timestamp_us << q.w << q.x << q.y << q.z);
        });
    }
    {
        auto& w = csv("attitude_euler", "timestamp_us,roll_deg,pitch_deg,yaw_deg");
        m_drone.attitude_euler_stream().add_listener([&w](T::EulerAngle e) {
            w.write(w.begin() << e.timestamp_us << e.roll_deg << e.pitch_deg << e.yaw_deg);
        });
    }
    {
        auto& w = csv("armed", "armed");
        m_drone.armed_stream().add_listener([this, &w](bool v) {
            if (changed("armed", to_str(v))) w.write(w.begin() << v);
        });
    }
    {
        auto& w = csv("in_air", "in_air");
        m_drone.in_air_stream().add_listener([this, &w](bool v) {
            if (changed("in_air", to_str(v))) w.write(w.begin() << v);
        });
    }

    // --- Streams nobody else uses: subscribe directly --------------------------------------
    {
        auto& w = csv("angular_velocity_body", "roll_rad_s,pitch_rad_s,yaw_rad_s");
        tel.subscribe_attitude_angular_velocity_body([&w](T::AngularVelocityBody a) {
            w.write(w.begin() << a.roll_rad_s << a.pitch_rad_s << a.yaw_rad_s);
        });
    }
    const std::string imu_header =
        "timestamp_us,ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s,mx_gauss,my_gauss,mz_gauss,temperature_degc";
    auto imu_row = [](CsvWriter& w, const T::Imu& i) {
        w.write(w.begin() << i.timestamp_us
            << i.acceleration_frd.forward_m_s2 << i.acceleration_frd.right_m_s2 << i.acceleration_frd.down_m_s2
            << i.angular_velocity_frd.forward_rad_s << i.angular_velocity_frd.right_rad_s << i.angular_velocity_frd.down_rad_s
            << i.magnetic_field_frd.forward_gauss << i.magnetic_field_frd.right_gauss << i.magnetic_field_frd.down_gauss
            << i.temperature_degc);
    };
    {
        auto& w = csv("imu", imu_header);  // HIGHRES_IMU: filtered, body FRD
        tel.subscribe_imu([&w, imu_row](T::Imu i) { imu_row(w, i); });
    }
    {
        auto& w = csv("scaled_imu", imu_header);
        tel.subscribe_scaled_imu([&w, imu_row](T::Imu i) { imu_row(w, i); });
    }
    {
        auto& w = csv("raw_imu", imu_header);
        tel.subscribe_raw_imu([&w, imu_row](T::Imu i) { imu_row(w, i); });
    }
    {
        auto& w = csv("odometry", "time_usec,frame_id,child_frame_id,x_m,y_m,z_m,qw,qx,qy,qz,vx_m_s,vy_m_s,vz_m_s,roll_rate_rad_s,pitch_rate_rad_s,yaw_rate_rad_s");
        tel.subscribe_odometry([&w](T::Odometry o) {
            w.write(w.begin() << o.time_usec << static_cast<int>(o.frame_id) << static_cast<int>(o.child_frame_id)
                << o.position_body.x_m << o.position_body.y_m << o.position_body.z_m
                << o.q.w << o.q.x << o.q.y << o.q.z
                << o.velocity_body.x_m_s << o.velocity_body.y_m_s << o.velocity_body.z_m_s
                << o.angular_velocity_body.roll_rad_s << o.angular_velocity_body.pitch_rad_s << o.angular_velocity_body.yaw_rad_s);
        });
    }
    {
        auto& w = csv("actuator_outputs", "active,out0,out1,out2,out3,out4,out5,out6,out7,out8,out9,out10,out11,out12,out13,out14,out15");
        tel.subscribe_actuator_output_status([&w](T::ActuatorOutputStatus a) {
            auto row = w.begin();
            row << a.active;
            row.padded(a.actuator, 16);
            w.write(row);
        });
    }
    {
        auto& w = csv("actuator_controls", "group,c0,c1,c2,c3,c4,c5,c6,c7");
        tel.subscribe_actuator_control_target([&w](T::ActuatorControlTarget a) {
            auto row = w.begin();
            row << a.group;
            row.padded(a.controls, 8);
            w.write(row);
        });
    }
    {
        auto& w = csv("global_position", "lat_deg,lon_deg,alt_amsl_m,alt_rel_m");
        tel.subscribe_position([&w](T::Position p) {
            w.write(w.begin() << p.latitude_deg << p.longitude_deg << p.absolute_altitude_m << p.relative_altitude_m);
        });
    }
    {
        auto& w = csv("home", "lat_deg,lon_deg,alt_amsl_m,alt_rel_m");
        tel.subscribe_home([this, &w](T::Position p) {
            const auto key = to_str(p.latitude_deg) + to_str(p.longitude_deg) + to_str(p.absolute_altitude_m);
            if (changed("home", key))
                w.write(w.begin() << p.latitude_deg << p.longitude_deg << p.absolute_altitude_m << p.relative_altitude_m);
        });
    }
    {
        auto& w = csv("ground_truth", "lat_deg,lon_deg,alt_amsl_m");
        tel.subscribe_ground_truth([&w](T::GroundTruth g) {
            w.write(w.begin() << g.latitude_deg << g.longitude_deg << g.absolute_altitude_m);
        });
    }
    {
        auto& w = csv("gps_info", "num_satellites,fix_type");
        tel.subscribe_gps_info([this, &w](T::GpsInfo g) {
            const auto key = to_str(g.num_satellites) + to_str(g.fix_type);
            if (changed("gps_info", key)) w.write(w.begin() << g.num_satellites << to_str(g.fix_type));
        });
    }
    {
        auto& w = csv("raw_gps", "timestamp_us,lat_deg,lon_deg,alt_amsl_m,hdop,vdop,velocity_m_s,cog_deg,alt_ellipsoid_m,h_unc_m,v_unc_m,vel_unc_m_s,heading_unc_deg,yaw_deg");
        tel.subscribe_raw_gps([&w](T::RawGps g) {
            w.write(w.begin() << g.timestamp_us << g.latitude_deg << g.longitude_deg << g.absolute_altitude_m
                << g.hdop << g.vdop << g.velocity_m_s << g.cog_deg << g.altitude_ellipsoid_m
                << g.horizontal_uncertainty_m << g.vertical_uncertainty_m << g.velocity_uncertainty_m_s
                << g.heading_uncertainty_deg << g.yaw_deg);
        });
    }
    {
        auto& w = csv("battery", "id,voltage_v,remaining_percent");
        tel.subscribe_battery([&w](T::Battery b) {
            w.write(w.begin() << b.id << b.voltage_v << b.remaining_percent);
        });
    }
    {
        auto& w = csv("flight_mode", "flight_mode");
        tel.subscribe_flight_mode([this, &w](T::FlightMode m) {
            const auto s = to_str(m);
            if (changed("flight_mode", s)) w.write(w.begin() << s);
        });
    }
    {
        auto& w = csv("landed_state", "landed_state");
        tel.subscribe_landed_state([this, &w](T::LandedState l) {
            const auto s = to_str(l);
            if (changed("landed_state", s)) w.write(w.begin() << s);
        });
    }
    {
        auto& w = csv("health", "gyro_cal_ok,accel_cal_ok,mag_cal_ok,local_position_ok,global_position_ok,home_position_ok,armable");
        tel.subscribe_health([this, &w](T::Health h) {
            const auto key = to_str(h.is_gyrometer_calibration_ok) + to_str(h.is_accelerometer_calibration_ok)
                + to_str(h.is_magnetometer_calibration_ok) + to_str(h.is_local_position_ok)
                + to_str(h.is_global_position_ok) + to_str(h.is_home_position_ok) + to_str(h.is_armable);
            if (changed("health", key))
                w.write(w.begin() << h.is_gyrometer_calibration_ok << h.is_accelerometer_calibration_ok
                    << h.is_magnetometer_calibration_ok << h.is_local_position_ok << h.is_global_position_ok
                    << h.is_home_position_ok << h.is_armable);
        });
    }
    {
        auto& w = csv("rc_status", "was_available_once,is_available,signal_strength_percent");
        tel.subscribe_rc_status([this, &w](T::RcStatus r) {
            const auto key = to_str(r.was_available_once) + to_str(r.is_available) + to_str(r.signal_strength_percent);
            if (changed("rc_status", key))
                w.write(w.begin() << r.was_available_once << r.is_available << r.signal_strength_percent);
        });
    }
    {
        auto& w = csv("status_text", "type,text");
        tel.subscribe_status_text([&w](T::StatusText s) {
            w.write(w.begin() << to_str(s.type) << s.text);
        });
    }
    {
        auto& w = csv("distance_sensor", "min_m,max_m,current_m");
        tel.subscribe_distance_sensor([&w](T::DistanceSensor d) {
            w.write(w.begin() << d.minimum_distance_m << d.maximum_distance_m << d.current_distance_m);
        });
    }
    {
        auto& w = csv("scaled_pressure", "timestamp_us,abs_pressure_hpa,diff_pressure_hpa,temperature_deg,diff_pressure_temperature_deg");
        tel.subscribe_scaled_pressure([&w](T::ScaledPressure p) {
            w.write(w.begin() << p.timestamp_us << p.absolute_pressure_hpa << p.differential_pressure_hpa
                << p.temperature_deg << p.differential_pressure_temperature_deg);
        });
    }

    // --- Commanded setpoints -----------------------------------------------------------------
    {
        auto& w = csv("setpoints", "type,a,b,c,yaw");
        m_drone.set_setpoint_listener([&w](const std::string& type, float a, float b, float c, float yaw) {
            w.write(w.begin() << type << a << b << c << yaw);
        });
    }
}

void FlightLogger::set_rates()
{
    // SITL MAVLink has plenty of bandwidth, but every message is a callback on MAVSDK's single
    // user thread, so keep CSV rates moderate; the PX4 ULog holds everything at full rate.
    // On the drone the PX4 link is a 57600 baud serial line (~5.7 kB/s) shared with VIO and
    // offboard traffic: keep it light there too.
    const double fast = m_sitl ? 30.0 : 10.0;
    const double imu = m_sitl ? 50.0 : 10.0;
    const double slow = m_sitl ? 5.0 : 1.0;

    auto& tel = m_drone.telemetry();
    const std::vector<std::pair<std::string, std::function<Telemetry::Result()>>> requests = {
        { "position_velocity_ned", [&]() { return tel.set_rate_position_velocity_ned(fast); } },
        { "attitude", [&]() { return tel.set_rate_attitude(fast); } },
        { "imu", [&]() { return tel.set_rate_imu(imu); } },
        { "scaled_imu", [&]() { return tel.set_rate_scaled_imu(m_sitl ? 10.0 : slow); } },
        { "raw_imu", [&]() { return tel.set_rate_raw_imu(m_sitl ? 10.0 : slow); } },
        { "odometry", [&]() { return tel.set_rate_odometry(fast); } },
        { "actuator_output_status", [&]() { return tel.set_rate_actuator_output_status(fast); } },
        { "actuator_control_target", [&]() { return tel.set_rate_actuator_control_target(fast); } },
        { "position", [&]() { return tel.set_rate_position(m_sitl ? 10.0 : slow); } },
        { "battery", [&]() { return tel.set_rate_battery(slow); } },
        { "gps_info", [&]() { return tel.set_rate_gps_info(slow); } },
        { "rc_status", [&]() { return tel.set_rate_rc_status(slow); } },
        { "distance_sensor", [&]() { return tel.set_rate_distance_sensor(m_sitl ? 10.0 : slow); } },
    };
    for (const auto& [name, request] : requests)
    {
        if (m_finished) return;
        const auto result = request();
        if (result != Telemetry::Result::Success)
        {
            event("rate", name + ": " + to_str(result) + " (PX4 may not stream it)");
        }
    }
    event("rate", "telemetry rates requested");
}

void FlightLogger::write_metadata()
{
    std::ofstream f(m_dir / "metadata.txt");
    const auto ws = data_root().parent_path();
    f << "start_local: " << timestamp_string() << '\n'
      << "connection: " << (m_sitl ? "SITL udp://:14540" : "serial") << '\n'
      << "workspace_branch: " << run("git -C '" + ws.string() + "' rev-parse --abbrev-ref HEAD 2>/dev/null") << '\n'
      << "workspace_commit: " << run("git -C '" + ws.string() + "' rev-parse --short HEAD 2>/dev/null") << '\n'
      << "workspace_dirty: " << (run("git -C '" + ws.string() + "' status --porcelain 2>/dev/null").empty() ? "no" : "yes") << '\n';

    if (m_finished) return;
    Info info{ m_drone.system() };
    if (const auto [res, v] = info.get_version(); res == Info::Result::Success)
    {
        f << "px4_version: " << v.flight_sw_major << '.' << v.flight_sw_minor << '.' << v.flight_sw_patch
          << " (" << v.flight_sw_git_hash << ")\n";
    }
    if (const auto [res, p] = info.get_product(); res == Info::Result::Success)
    {
        f << "product: " << p.vendor_name << ' ' << p.product_name << '\n';
    }
    f << "frames: NED world (north, east, down), FRD body; t_unix is wall clock, t is seconds since logger start\n";
}

void FlightLogger::start_rosbag()
{
    if (const char* env = std::getenv("SUAVE_ROSBAG"); env && std::string(env) == "0")
    {
        event("rosbag", "disabled (SUAVE_ROSBAG=0)");
        return;
    }
    m_rosbag = std::make_shared<SystemTask>(std::vector<std::string>{
        "source /opt/ros/humble/setup.bash",
        "ros2 bag record -a --include-hidden-topics -o \"" + (m_dir / "rosbag").string() + "\""
    });
    m_rosbag->start_in_thread();
    event("rosbag", "recording all ROS 2 topics");
}

void FlightLogger::copy_px4_ulog()
{
    const char* env = std::getenv("PX4_DIR");
    const char* home = std::getenv("HOME");
    const fs::path px4_dir = env ? fs::path(env) : fs::path(home ? home : ".") / "Dev" / "PX4-Autopilot";
    const auto log_root = px4_dir / "build" / "px4_sitl_default" / "rootfs" / "log";

    std::error_code ec;
    if (!fs::exists(log_root, ec)) return;

    fs::path newest{};
    fs::file_time_type newest_time{};
    for (const auto& entry : fs::recursive_directory_iterator(log_root, ec))
    {
        if (entry.path().extension() != ".ulg") continue;
        const auto t = entry.last_write_time(ec);
        if (t >= m_start_file_time && (newest.empty() || t > newest_time))
        {
            newest = entry.path();
            newest_time = t;
        }
    }
    if (newest.empty())
    {
        event("ulog", "no PX4 log written during this flight found in " + log_root.string());
        return;
    }
    fs::copy_file(newest, m_dir / ("px4_" + newest.filename().string()), fs::copy_options::overwrite_existing, ec);
    event("ulog", ec ? "copy failed: " + ec.message() : "copied " + newest.string());
}

void FlightLogger::finish()
{
    if (m_finished.exchange(true)) return;

    event("stop", "logger finishing");
    {
        std::lock_guard<std::mutex> lock{ s_active_mutex };
        if (s_active == this) s_active = nullptr;
    }
    if (m_setup_thread.joinable()) m_setup_thread.join();
    if (m_rosbag) m_rosbag->stop();
    if (m_sitl) copy_px4_ulog();

    std::error_code ec;
    fs::remove(m_marker, ec);

    std::ofstream summary(m_dir / "summary.txt");
    const double duration = std::chrono::duration<double>(CsvWriter::Clock::now() - m_origin).count();
    summary << "duration_s: " << duration << "\nrows per file:\n";
    {
        std::lock_guard<std::mutex> lock{ m_writers_mutex };
        for (auto& [name, writer] : m_writers)
        {
            writer->flush();
            summary << "  " << name << ".csv: " << writer->rows() << '\n';
        }
    }
    suave_log << "Flight data saved in " << m_dir.string() << std::endl;
}

// SuaveDisturbance: Gazebo Harmonic (gz-sim8) world system that applies wind and external
// disturbances to the PX4 vehicle and logs them, together with ground truth, next to the
// flight data written by suave_main.
//
// Commands arrive as gz.msgs.StringMsg on /suave/disturbance (all vectors in NED, SI units):
//   wind <n> <e> <d>           steady wind velocity (direction the air moves toward), m/s
//   turb <sigma> [L] [sigma_d] turbulence: first-order Gauss-Markov (Dryden-like) per axis,
//                              sigma m/s horizontal, L length scale m (20), sigma_d vertical (sigma/2)
//   gust <n> <e> <d> <dur>     1-cosine discrete gust peaking at (n,e,d) m/s, lasting dur s
//   push <n> <e> <d> <dur>     external force, N, for dur s (world NED)
//   torque <r> <p> <y> <dur>   external torque, N m, about body FRD axes, for dur s
//   drag <cd> <a_xy> <a_z>     drag coefficient and reference areas (m^2) of the body
//   calm                       remove every disturbance
//   seed <n>                   reseed the turbulence generator
//   status                     reply with the current state on /suave/disturbance/status
//
// Wind acts through body drag: D(u) = -1/2 rho |u| (Cd A) u, applied as D(v - w) - D(v),
// i.e. only the extra force caused by the wind. With no wind the vehicle is untouched, so
// baseline flights are identical to stock PX4 SITL.
//
// Gazebo's world frame is ENU and the x500 body frame is FLU; everything user-facing and
// logged is converted to NED / FRD to match PX4 and suave_main.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace fs = std::filesystem;
using gz::math::Vector3d;

namespace suave
{

namespace
{
// World ENU <-> NED for vectors
// (+ 0.0 turns -0 into 0 so logs and status read cleanly)
Vector3d EnuToNed(const Vector3d& v) { return { v.Y() + 0.0, v.X() + 0.0, -v.Z() + 0.0 }; }
Vector3d NedToEnu(const Vector3d& v) { return { v.Y() + 0.0, v.X() + 0.0, -v.Z() + 0.0 }; }

double Seconds(const std::chrono::steady_clock::duration& d)
{
    return std::chrono::duration<double>(d).count();
}

double UnixNow()
{
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string Stamp()
{
    const auto now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

// Body drag force for relative airflow u (world frame), per-axis Cd*A.
Vector3d Drag(const Vector3d& u, const Vector3d& cda, double rho)
{
    const double speed = u.Length();
    return { -0.5 * rho * speed * cda.X() * u.X(),
             -0.5 * rho * speed * cda.Y() * u.Y(),
             -0.5 * rho * speed * cda.Z() * u.Z() };
}
}

class SuaveDisturbance : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate
{
public:
    ~SuaveDisturbance() override
    {
        CloseLog();
    }

    void Configure(const gz::sim::Entity&, const std::shared_ptr<const sdf::Element>& sdf,
                   gz::sim::EntityComponentManager&, gz::sim::EventManager&) override
    {
        auto get = [&](const char* name, auto fallback) {
            using T = decltype(fallback);
            return sdf && sdf->HasElement(name) ? sdf->Get<T>(name) : fallback;
        };
        m_modelName = get("model_name", std::string("x500_0"));
        m_linkName = get("link_name", std::string("base_link"));
        m_rho = get("air_density", 1.225);
        m_cd = get("drag_coefficient", 1.0);
        m_areaXY = get("area_xy", 0.05);
        m_areaZ = get("area_z", 0.12);
        m_logRateHz = get("log_rate_hz", 100.0);
        m_rng.seed(static_cast<unsigned>(get("seed", 1)));

        std::string dataDir = get("data_dir", std::string(""));
        if (dataDir.empty())
        {
            if (const char* env = std::getenv("SUAVE_DATA_DIR"); env && *env) dataDir = env;
            else dataDir = std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/Dev/dronecpp-ros2_ws/data";
        }
        m_dataRoot = dataDir;

        m_node.Subscribe("/suave/disturbance", &SuaveDisturbance::OnCommand, this);
        m_statusPub = m_node.Advertise<gz::msgs::StringMsg>("/suave/disturbance/status");

        gzmsg << "[SuaveDisturbance] model [" << m_modelName << "::" << m_linkName << "], commands on "
              << "/suave/disturbance, logging under " << m_dataRoot << std::endl;
    }

    void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override
    {
        if (info.paused) return;

        const double t = Seconds(info.simTime);
        const double dt = Seconds(info.dt);

        ProcessCommands(t);
        UpdateLogTarget();

        if (!ResolveLink(ecm)) return;

        const auto pose = m_link.WorldPose(ecm);
        const auto vel = m_link.WorldLinearVelocity(ecm);
        if (!pose || !vel) return;  // velocity component appears one step after enabling

        // ---- wind (ENU) ------------------------------------------------------------------
        UpdateTurbulence(dt);
        Vector3d gust{};
        for (const auto& g : m_gusts)
        {
            const double tau = t - g.start;
            if (tau >= 0 && tau <= g.duration)
                gust += g.amplitude * (0.5 * (1.0 - std::cos(2.0 * M_PI * tau / g.duration)));
        }
        const Vector3d wind = m_meanWind + m_turb + gust;

        const Vector3d cda{ m_cd * m_areaXY, m_cd * m_areaXY, m_cd * m_areaZ };
        const Vector3d windForce = Drag(*vel - wind, cda, m_rho) - Drag(*vel, cda, m_rho);

        // ---- external force / torque -------------------------------------------------------
        Vector3d pushForce{};
        for (const auto& p : m_pushes)
            if (t >= p.start && t <= p.start + p.duration) pushForce += p.value;

        Vector3d torqueFrd{};
        for (const auto& q : m_torques)
            if (t >= q.start && t <= q.start + q.duration) torqueFrd += q.value;
        const Vector3d torqueFlu{ torqueFrd.X(), -torqueFrd.Y(), -torqueFrd.Z() };
        const Vector3d torqueWorld = pose->Rot().RotateVector(torqueFlu);

        const Vector3d totalForce = windForce + pushForce;
        if (totalForce != Vector3d::Zero || torqueWorld != Vector3d::Zero)
        {
            m_link.AddWorldWrench(ecm, totalForce, torqueWorld);
        }

        Prune(t);
        Log(ecm, info, *pose, *vel, wind, gust, windForce, pushForce, torqueFrd);
    }

private:
    struct Timed
    {
        double start;
        double duration;
        Vector3d value;
    };
    struct Gust
    {
        double start;
        double duration;
        Vector3d amplitude;
    };

    // ------------------------------------------------------------------ commands
    void OnCommand(const gz::msgs::StringMsg& msg)
    {
        std::lock_guard<std::mutex> lock{ m_cmdMutex };
        m_commands.push_back(msg.data());
    }

    void Reply(const std::string& text)
    {
        gz::msgs::StringMsg msg;
        msg.set_data(text);
        m_statusPub.Publish(msg);
        gzmsg << "[SuaveDisturbance] " << text << std::endl;
    }

    void ProcessCommands(double t)
    {
        std::deque<std::string> commands;
        {
            std::lock_guard<std::mutex> lock{ m_cmdMutex };
            commands.swap(m_commands);
        }
        for (const auto& line : commands)
        {
            const std::string result = Execute(line, t);
            Reply(result);
            LogEvent(t, line, result);
        }
    }

    std::string Execute(const std::string& line, double t)
    {
        std::istringstream in{ line };
        std::string cmd;
        in >> cmd;
        double a{}, b{}, c{}, d{};
        std::ostringstream out;
        out << std::setprecision(4);

        if (cmd == "wind")
        {
            std::string first;
            in >> first;
            if (first == "off") { m_meanWind = Vector3d::Zero; return "wind off"; }
            std::istringstream rest{ first + " " + std::string(std::istreambuf_iterator<char>(in), {}) };
            if (!(rest >> a >> b >> c)) return "error: usage wind <n> <e> <d> (m/s)";
            m_meanWind = NedToEnu({ a, b, c });
            out << "wind NED = (" << a << ", " << b << ", " << c << ") m/s, " << Vector3d(a, b, c).Length() << " m/s";
            return out.str();
        }
        if (cmd == "turb")
        {
            std::string first;
            in >> first;
            if (first == "off" || first == "0") { m_sigma = 0; m_turb = Vector3d::Zero; return "turbulence off"; }
            try { a = std::stod(first); } catch (...) { return "error: usage turb <sigma> [L] [sigma_d]"; }
            double L = 20.0, sv = a / 2.0;
            in >> L >> sv;
            if (a < 0 || L <= 0 || sv < 0) return "error: sigma >= 0, L > 0";
            m_sigma = a; m_sigmaV = sv; m_lengthScale = L;
            out << "turbulence sigma " << a << " m/s (vertical " << sv << "), L " << L << " m";
            return out.str();
        }
        if (cmd == "gust" || cmd == "push" || cmd == "torque")
        {
            if (!(in >> a >> b >> c >> d) || d <= 0) return "error: usage " + cmd + " <x> <y> <z> <duration_s>";
            if (cmd == "gust")
            {
                m_gusts.push_back({ t, d, NedToEnu({ a, b, c }) });
                out << "gust peak NED (" << a << ", " << b << ", " << c << ") m/s over " << d << " s";
            }
            else if (cmd == "push")
            {
                m_pushes.push_back({ t, d, NedToEnu({ a, b, c }) });
                out << "push NED (" << a << ", " << b << ", " << c << ") N for " << d << " s";
            }
            else
            {
                m_torques.push_back({ t, d, { a, b, c } });
                out << "torque FRD (" << a << ", " << b << ", " << c << ") N m for " << d << " s";
            }
            return out.str();
        }
        if (cmd == "drag")
        {
            if (!(in >> a >> b >> c) || a < 0 || b < 0 || c < 0) return "error: usage drag <cd> <area_xy> <area_z>";
            m_cd = a; m_areaXY = b; m_areaZ = c;
            out << "drag cd " << a << ", area_xy " << b << " m^2, area_z " << c << " m^2";
            return out.str();
        }
        if (cmd == "calm")
        {
            m_meanWind = Vector3d::Zero; m_turb = Vector3d::Zero; m_sigma = 0; m_sigmaV = 0;
            m_gusts.clear(); m_pushes.clear(); m_torques.clear();
            return "calm: all disturbances removed";
        }
        if (cmd == "seed")
        {
            unsigned s{};
            if (!(in >> s)) return "error: usage seed <n>";
            m_rng.seed(s);
            return "seed " + std::to_string(s);
        }
        if (cmd == "status")
        {
            const auto w = EnuToNed(m_meanWind);
            out << "model " << m_modelName << (m_link.Entity() != gz::sim::kNullEntity ? " (found)" : " (not spawned yet)")
                << " | wind NED (" << w.X() << ", " << w.Y() << ", " << w.Z() << ") m/s"
                << " | turb sigma " << m_sigma << "/" << m_sigmaV << " L " << m_lengthScale
                << " | active gusts " << m_gusts.size() << ", pushes " << m_pushes.size() << ", torques " << m_torques.size()
                << " | drag cd " << m_cd << " A " << m_areaXY << "/" << m_areaZ
                << " | log " << (m_logPath.empty() ? std::string("none") : m_logPath.string());
            return out.str();
        }
        return "error: unknown command '" + cmd + "' (wind, turb, gust, push, torque, drag, calm, seed, status)";
    }

    // ------------------------------------------------------------------ dynamics
    void UpdateTurbulence(double dt)
    {
        if (m_sigma <= 0 && m_sigmaV <= 0) { m_turb = Vector3d::Zero; return; }
        // Time constant from the length scale and mean wind speed (Taylor's frozen turbulence);
        // floor at 1 m/s so hover in calm air still decorrelates.
        const double V = std::max(1.0, m_meanWind.Length());
        const double T = m_lengthScale / V;
        const double alpha = std::exp(-dt / T);
        const double k = std::sqrt(1.0 - alpha * alpha);
        m_turb.X(alpha * m_turb.X() + m_sigma * k * m_normal(m_rng));
        m_turb.Y(alpha * m_turb.Y() + m_sigma * k * m_normal(m_rng));
        m_turb.Z(alpha * m_turb.Z() + m_sigmaV * k * m_normal(m_rng));
    }

    void Prune(double t)
    {
        auto expiredT = [t](const Timed& x) { return t > x.start + x.duration; };
        auto expiredG = [t](const Gust& x) { return t > x.start + x.duration; };
        m_pushes.erase(std::remove_if(m_pushes.begin(), m_pushes.end(), expiredT), m_pushes.end());
        m_torques.erase(std::remove_if(m_torques.begin(), m_torques.end(), expiredT), m_torques.end());
        m_gusts.erase(std::remove_if(m_gusts.begin(), m_gusts.end(), expiredG), m_gusts.end());
    }

    bool ResolveLink(gz::sim::EntityComponentManager& ecm)
    {
        if (m_link.Entity() != gz::sim::kNullEntity && ecm.HasEntity(m_link.Entity()))
            return true;

        m_link = gz::sim::Link();
        const auto modelEntity = ecm.EntityByComponents(gz::sim::components::Name(m_modelName),
                                                        gz::sim::components::Model());
        if (modelEntity == gz::sim::kNullEntity) return false;

        const gz::sim::Model model{ modelEntity };
        const auto linkEntity = model.LinkByName(ecm, m_linkName);
        if (linkEntity == gz::sim::kNullEntity) return false;

        m_link = gz::sim::Link(linkEntity);
        m_link.EnableVelocityChecks(ecm, true);
        m_link.EnableAccelerationChecks(ecm, true);
        gzmsg << "[SuaveDisturbance] attached to " << m_modelName << "::" << m_linkName << std::endl;
        return false;  // velocities are available from the next step
    }

    // ------------------------------------------------------------------ logging
    // suave_main writes <data>/.active_flight with its flight folder; log there while it
    // exists, otherwise into <data>/sim_sessions/.
    void UpdateLogTarget()
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - m_lastMarkerCheck < std::chrono::milliseconds(500)) return;
        m_lastMarkerCheck = now;

        fs::path target{};
        std::error_code ec;
        const auto marker = m_dataRoot / ".active_flight";
        if (fs::exists(marker, ec))
        {
            std::ifstream f(marker);
            std::string dir;
            std::getline(f, dir);
            if (!dir.empty() && fs::is_directory(dir, ec)) target = fs::path(dir) / "disturbance.csv";
        }
        if (target.empty())
        {
            if (!m_sessionPath.empty()) target = m_sessionPath;
            else
            {
                fs::create_directories(m_dataRoot / "sim_sessions", ec);
                m_sessionPath = m_dataRoot / "sim_sessions" / ("sim_" + Stamp() + "_disturbance.csv");
                target = m_sessionPath;
            }
        }
        if (target != m_logPath) OpenLog(target);
    }

    void OpenLog(const fs::path& path)
    {
        CloseLog();
        const bool exists = fs::exists(path);
        m_log.open(path, std::ofstream::out | std::ofstream::app);
        if (!m_log)
        {
            gzerr << "[SuaveDisturbance] cannot write " << path << std::endl;
            return;
        }
        m_logPath = path;
        if (!exists)
        {
            m_log << "t_unix,t_sim,"
                     "n_m,e_m,d_m,vn_m_s,ve_m_s,vd_m_s,an_m_s2,ae_m_s2,ad_m_s2,"
                     "roll_deg,pitch_deg,yaw_deg,p_rad_s,q_rad_s,r_rad_s,"
                     "wind_n,wind_e,wind_d,mean_wind_n,mean_wind_e,mean_wind_d,turb_n,turb_e,turb_d,gust_n,gust_e,gust_d,"
                     "airspeed_n,airspeed_e,airspeed_d,"
                     "f_wind_n,f_wind_e,f_wind_d,f_push_n,f_push_e,f_push_d,tau_x_frd,tau_y_frd,tau_z_frd,"
                     "cd,area_xy,area_z\n";
        }
        m_log << std::setprecision(9);
        const auto events = path.parent_path() / "disturbance_events.csv";
        const bool eventsExist = fs::exists(events);
        m_events.open(events, std::ofstream::out | std::ofstream::app);
        if (!eventsExist) m_events << "t_unix,t_sim,command,result\n";
        gzmsg << "[SuaveDisturbance] logging to " << path << std::endl;
    }

    void CloseLog()
    {
        if (m_log.is_open()) m_log.close();
        if (m_events.is_open()) m_events.close();
        m_logPath.clear();
    }

    void LogEvent(double t, const std::string& cmd, const std::string& result)
    {
        if (!m_events.is_open()) return;
        auto quote = [](std::string s) {
            for (auto& ch : s) if (ch == '"') ch = '\'';
            return "\"" + s + "\"";
        };
        m_events << std::fixed << std::setprecision(6) << UnixNow() << ',' << t << ','
                 << quote(cmd) << ',' << quote(result) << '\n' << std::defaultfloat;
        m_events.flush();
    }

    void Log(gz::sim::EntityComponentManager& ecm, const gz::sim::UpdateInfo& info,
             const gz::math::Pose3d& pose, const Vector3d& velEnu, const Vector3d& windEnu,
             const Vector3d& gustEnu, const Vector3d& windForceEnu, const Vector3d& pushEnu,
             const Vector3d& torqueFrd)
    {
        if (!m_log.is_open() || m_logRateHz <= 0) return;
        const double t = Seconds(info.simTime);
        if (t < m_nextLog) return;
        m_nextLog = t + 1.0 / m_logRateHz;

        const auto accEnu = m_link.WorldLinearAcceleration(ecm).value_or(Vector3d::Zero);
        const auto omegaWorld = m_link.WorldAngularVelocity(ecm).value_or(Vector3d::Zero);

        // Attitude: R_ned_frd = T * R_enu_flu * B, with T = ENU->NED and B = FRD->FLU.
        const gz::math::Matrix3d R(pose.Rot());
        const gz::math::Matrix3d T(0, 1, 0, 1, 0, 0, 0, 0, -1);
        const gz::math::Matrix3d B(1, 0, 0, 0, -1, 0, 0, 0, -1);
        const gz::math::Matrix3d Rn = T * R * B;
        const double roll = std::atan2(Rn(2, 1), Rn(2, 2));
        const double pitch = -std::asin(std::clamp(Rn(2, 0), -1.0, 1.0));
        const double yaw = std::atan2(Rn(1, 0), Rn(0, 0));
        const Vector3d omegaFlu = pose.Rot().RotateVectorReverse(omegaWorld);

        const auto p = EnuToNed(pose.Pos());
        const auto v = EnuToNed(velEnu);
        const auto a = EnuToNed(accEnu);
        const auto w = EnuToNed(windEnu);
        const auto wm = EnuToNed(m_meanWind);
        const auto wt = EnuToNed(m_turb);
        const auto wg = EnuToNed(gustEnu);
        const auto air = EnuToNed(velEnu - windEnu);
        const auto fw = EnuToNed(windForceEnu);
        const auto fp = EnuToNed(pushEnu);
        constexpr double deg = 180.0 / M_PI;

        m_log << std::fixed << std::setprecision(6) << UnixNow() << ',' << t << std::defaultfloat << std::setprecision(9)
              << ',' << p.X() << ',' << p.Y() << ',' << p.Z()
              << ',' << v.X() << ',' << v.Y() << ',' << v.Z()
              << ',' << a.X() << ',' << a.Y() << ',' << a.Z()
              << ',' << roll * deg << ',' << pitch * deg << ',' << yaw * deg
              << ',' << omegaFlu.X() << ',' << -omegaFlu.Y() << ',' << -omegaFlu.Z()
              << ',' << w.X() << ',' << w.Y() << ',' << w.Z()
              << ',' << wm.X() << ',' << wm.Y() << ',' << wm.Z()
              << ',' << wt.X() << ',' << wt.Y() << ',' << wt.Z()
              << ',' << wg.X() << ',' << wg.Y() << ',' << wg.Z()
              << ',' << air.X() << ',' << air.Y() << ',' << air.Z()
              << ',' << fw.X() << ',' << fw.Y() << ',' << fw.Z()
              << ',' << fp.X() << ',' << fp.Y() << ',' << fp.Z()
              << ',' << torqueFrd.X() << ',' << torqueFrd.Y() << ',' << torqueFrd.Z()
              << ',' << m_cd << ',' << m_areaXY << ',' << m_areaZ << '\n';
        if (++m_rowsSinceFlush >= 100) { m_log.flush(); m_rowsSinceFlush = 0; }
    }

    // configuration
    std::string m_modelName;
    std::string m_linkName;
    double m_rho{ 1.225 };
    double m_cd{ 1.0 };
    double m_areaXY{ 0.05 };
    double m_areaZ{ 0.12 };
    double m_logRateHz{ 100.0 };
    fs::path m_dataRoot;

    // state (ENU)
    gz::sim::Link m_link{};
    Vector3d m_meanWind{};
    Vector3d m_turb{};
    double m_sigma{ 0 };
    double m_sigmaV{ 0 };
    double m_lengthScale{ 20.0 };
    std::vector<Gust> m_gusts;
    std::vector<Timed> m_pushes;
    std::vector<Timed> m_torques;
    std::mt19937 m_rng{ 1 };
    std::normal_distribution<double> m_normal{ 0.0, 1.0 };

    // transport
    gz::transport::Node m_node;
    gz::transport::Node::Publisher m_statusPub;
    std::mutex m_cmdMutex;
    std::deque<std::string> m_commands;

    // logging
    std::ofstream m_log;
    std::ofstream m_events;
    fs::path m_logPath;
    fs::path m_sessionPath;
    double m_nextLog{ 0 };
    int m_rowsSinceFlush{ 0 };
    std::chrono::steady_clock::time_point m_lastMarkerCheck{};
};

}  // namespace suave

GZ_ADD_PLUGIN(suave::SuaveDisturbance, gz::sim::System,
              suave::SuaveDisturbance::ISystemConfigure,
              suave::SuaveDisturbance::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(suave::SuaveDisturbance, "suave::SuaveDisturbance")

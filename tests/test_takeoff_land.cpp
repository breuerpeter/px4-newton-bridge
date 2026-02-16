//
// SITL integration test: arm, takeoff, hover, land.
//
// Connects to PX4 via MAVSDK on the offboard UDP port (14540),
// commands a takeoff/land sequence while monitoring stability,
// and reports real-time factor (RTF) and simulation speed.
//
// Expects PX4 SITL + Newton bridge to already be running.
//

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <atomic>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::steady_clock;
using std::this_thread::sleep_for;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
constexpr float TAKEOFF_ALT_M = 5.0f;
constexpr float MAX_TILT_DEG = 60.0f;
constexpr int HOVER_ATTITUDE_MSGS = 250; // ~5 s at 50 Hz ATTITUDE stream
constexpr double AUTOPILOT_TIMEOUT_S = 60.0;
constexpr auto WALL_TIMEOUT = seconds(120);
constexpr double ATTITUDE_STREAM_HZ = 50.0; // PX4 SITL default
constexpr double SIM_PHYSICS_HZ = 250.0; // Newton bridge sim_dt = 0.004 s

int main(int argc, char** argv)
{
    // Disable stdout buffering so output is visible in containers without a TTY
    std::cout << std::unitbuf;

    const std::string conn_url =
        (argc > 1) ? argv[1] : "udpin://0.0.0.0:14540";

    std::cout << "============================================================\n"
              << "PX4 Newton SITL -- Takeoff / Land Integration Test\n"
              << "============================================================\n"
              << "Connecting to " << conn_url << " ...\n";

    // ---- Connect ----
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};

    ConnectionResult conn_result = mavsdk.add_any_connection(conn_url);
    if (conn_result != ConnectionResult::Success) {
        std::cerr << "FAIL: Connection failed: " << conn_result << "\n";
        return 1;
    }

    std::cout << "Waiting for PX4 autopilot ...\n";
    auto system = mavsdk.first_autopilot(AUTOPILOT_TIMEOUT_S);
    if (!system) {
        std::cerr << "FAIL: No autopilot found within "
                  << AUTOPILOT_TIMEOUT_S << " s\n";
        return 1;
    }
    std::cout << "  Autopilot connected\n";

    auto telemetry = Telemetry{system.value()};
    auto action = Action{system.value()};

    // ---- Telemetry subscriptions ----
    std::atomic<float> alt_m{0.0f};
    std::atomic<float> roll_deg{0.0f};
    std::atomic<float> pitch_deg{0.0f};
    std::atomic<int> attitude_count{0};
    std::atomic<bool> tilt_fail{false};

    telemetry.subscribe_position([&](Telemetry::Position pos) {
        alt_m.store(pos.relative_altitude_m);
    });

    telemetry.subscribe_attitude_euler([&](Telemetry::EulerAngle euler) {
        roll_deg.store(euler.roll_deg);
        pitch_deg.store(euler.pitch_deg);
        attitude_count.fetch_add(1);
        if (std::abs(euler.roll_deg) > MAX_TILT_DEG ||
            std::abs(euler.pitch_deg) > MAX_TILT_DEG) {
            tilt_fail.store(true);
        }
    });

    // ---- Wait for home position (minimum readiness indicator) ----
    std::cout << "Waiting for vehicle to be ready ...\n";
    auto health_start = steady_clock::now();
    while (!telemetry.health().is_home_position_ok) {
        if (steady_clock::now() - health_start > WALL_TIMEOUT) {
            std::cerr << "FAIL: Home position not set within timeout\n";
            return 1;
        }
        sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "  Home position set\n";

    // ---- Begin timed section for RTF measurement ----
    auto wall_start = steady_clock::now();
    int att_start = attitude_count.load();

    auto check = [&](const char* phase) -> bool {
        if (tilt_fail.load()) {
            std::cerr << "FAIL: Excessive tilt during " << phase
                      << " (roll=" << roll_deg.load()
                      << " pitch=" << pitch_deg.load() << ")\n";
            return false;
        }
        if (steady_clock::now() - wall_start > WALL_TIMEOUT) {
            std::cerr << "FAIL: Wall-clock timeout during " << phase << "\n";
            return false;
        }
        return true;
    };

    // ---- Arm (retry until PX4's commander allows it) ----
    std::cout << "Arming ...\n";
    Action::Result arm_result;
    while (true) {
        arm_result = action.arm();
        if (arm_result == Action::Result::Success) break;
        if (steady_clock::now() - health_start > WALL_TIMEOUT) {
            std::cerr << "FAIL: Arm failed after timeout: " << arm_result << "\n";
            return 1;
        }
        sleep_for(seconds(1));
    }

    // ---- Takeoff ----
    action.set_takeoff_altitude(TAKEOFF_ALT_M);
    std::cout << "Taking off to " << TAKEOFF_ALT_M << " m ...\n";
    const auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "FAIL: Takeoff failed: " << takeoff_result << "\n";
        return 1;
    }

    // Wait until takeoff altitude reached
    while (alt_m.load() < TAKEOFF_ALT_M * 0.6f) {
        if (!check("takeoff")) return 1;
        sleep_for(std::chrono::milliseconds(50));
    }
    std::cout << "  Takeoff altitude reached: " << alt_m.load() << " m\n";

    // ---- Hover ----
    std::cout << "Hovering ...\n";
    int hover_start = attitude_count.load();
    while (attitude_count.load() - hover_start < HOVER_ATTITUDE_MSGS) {
        if (!check("hover")) return 1;
        sleep_for(std::chrono::milliseconds(50));
    }
    std::cout << "  Hover complete\n";

    // ---- Land ----
    std::cout << "Landing ...\n";
    const auto land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "FAIL: Land failed: " << land_result << "\n";
        return 1;
    }

    while (telemetry.in_air()) {
        if (!check("landing")) return 1;
        sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "  Landed (alt=" << alt_m.load() << " m)\n";

    // Wait for auto-disarm
    sleep_for(seconds(3));

    // ---- End timed section ----
    auto wall_end = steady_clock::now();
    int att_end = attitude_count.load();

    double wall_s =
        std::chrono::duration<double>(wall_end - wall_start).count();
    int att_total = att_end - att_start;
    double sim_s = att_total / ATTITUDE_STREAM_HZ;
    double rtf = sim_s / wall_s;
    double sim_fps = (sim_s * SIM_PHYSICS_HZ) / wall_s;

    std::cout
        << "\n------------------------------------------------------------\n"
        << "RESULT: PASS\n\n"
        << "Speed Statistics:\n"
        << "  Real-time factor (RTF) : " << rtf << "x\n"
        << "  Sim FPS (250 Hz basis) : " << static_cast<int>(sim_fps)
        << " steps/s\n"
        << "  Sim time               : " << sim_s << " s\n"
        << "  Wall time              : " << wall_s << " s\n"
        << "  ATTITUDE messages      : " << att_total << "\n"
        << "  ATTITUDE msg rate      : " << att_total / wall_s << " Hz\n"
        << "------------------------------------------------------------\n";

    return 0;
}

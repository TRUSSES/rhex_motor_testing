#include "cubemars_pi3hat.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
namespace {

// Motor side info for direction control
struct MotorInfo {
    int id;
    CubemarsPi3Hat* motor;
    bool is_left_side;  // true = left side (positive velocity, CCW), false = right side (negative velocity, CW)
    float home_pos = 0.0f;  // Home position in radians
    float amp = 0.0f; // safe amplitude for this motor
    float phase_off = 0.0f; // phase offset for this motor
};


void CaptureHome(std::vector<MotorInfo>& tripod,
                 float kp_hold = 0.0f,
                 float kd_hold = 0.5f,
                std::chrono::milliseconds hold_time = std::chrono::milliseconds(300)) {

    const auto period = std::chrono::milliseconds(10);
    // 1) Prime feedback: send a few "do nothing" frames so getPosition() is fresh.
    for (int k = 0; k < 20; ++k) {  // ~200ms
        for (auto& info : tripod) {
        float p = info.motor->getPosition();                 // whatever is currently cached
        info.motor->sendCommandMITMode(p, 0.0f, 0.0f, 0.5f, 0.0f); // kp=0 => no pull
        }
        std::this_thread::sleep_for(period);
    }

    for (auto& info : tripod) {
        info.home_pos = info.motor->getPosition();
        std::cout << "ID " << info.id << " Initial read position: " << 
        info.home_pos << " rad" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // hold current position for a short time to capture home
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < hold_time) {
        for (auto& info : tripod) {
            // Hold at whatever it currently is by using v_des=0 and a small damping.
            // We still set home_pos=0 byt with kp=0 so it won't pull to zero.
            info.motor->sendCommandMITMode(info.home_pos, 0.0f, kp_hold, kd_hold, 0.0f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Now read absolute positions as home
    for (auto& info : tripod) {
        float pos = info.motor->getPosition();
        info.home_pos = pos;
        std::cout << "ID " << info.id << " Captured home position: " << pos << " rad" << std::endl;
    }
}

static float ComputeSafeAmplitude(float home, float A_cmd) {
  constexpr float P_MIN = -12.5f;
  constexpr float P_MAX = +12.5f;
  constexpr float margin = 0.25f; // keep away from clamp edge

  float up = (P_MAX - margin) - home;
  float dn = home - (P_MIN + margin);
  float A_safe = std::max(0.0f, std::min(up, dn));
  return std::min(A_cmd, A_safe);
}

// Stop all motors in a tripod
void StopTripod(std::vector<MotorInfo>& tripod, float kd) {
    for (auto& info : tripod) {
        info.motor->sendCommandMITMode(0.0f, 0.0f, 0.0f, kd, 0.0f);
    }
}

static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// Run a single tripod with correct direction based on motor mounting side
// other_tripod is stopped while this tripod runs
void RunTripod(std::vector<MotorInfo>& tripod,
               std::vector<MotorInfo>& other_tripod,
               float velocity,
               std::chrono::milliseconds duration,
               float kp,
               float kd) {
    auto start = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(10);

    while (std::chrono::steady_clock::now() - start < duration) {
        // Run active tripod
        for (auto& info : tripod) {
            float dir = info.is_left_side ? -1.0f : 1.0f;
            info.motor->sendCommandMITMode(0.0f, velocity * dir, kp, kd, 0.0f);
        }
        // Keep other tripod stopped
        StopTripod(other_tripod, kd);
        std::this_thread::sleep_for(period);
    }
}

}  // namespace

int main(int argc, char** argv) {
    int can_bus = 4;  // Default to CAN bus 4 (JC5 connector)
    if (argc > 1) {
        can_bus = std::atoi(argv[1]);
    }

    // Configure Pi3Hat CAN bus for CubeMars motors
    mjbots::pi3hat::Pi3Hat::Configuration config;
    config.can[4].slow_bitrate = 1000000;     // 1 Mbps for CubeMars
    config.can[4].fdcan_frame = false;        // Standard CAN frames
    config.can[4].bitrate_switch = false;     // No FD bitrate switching
    // Disable unused CAN buses
    config.can[0].slow_bitrate = 0;
    config.can[1].slow_bitrate = 0;
    config.can[2].slow_bitrate = 0;
    config.can[3].slow_bitrate = 0;

    mjbots::pi3hat::Pi3Hat pi3hat(config);

    // Create motors by ID
    // Left side (mounted): 10, 14, 13
    // Right side (mounted): 12, 11, 15
    CubemarsPi3Hat motor_10(10, can_bus, &pi3hat);
    CubemarsPi3Hat motor_11(11, can_bus, &pi3hat);
    CubemarsPi3Hat motor_12(12, can_bus, &pi3hat);
    CubemarsPi3Hat motor_13(13, can_bus, &pi3hat);
    CubemarsPi3Hat motor_14(14, can_bus, &pi3hat);
    CubemarsPi3Hat motor_15(15, can_bus, &pi3hat);

    // Tripod configuration for walking gait
    // Left tripod: 13 (left), 10 (left), 11 (right)
    // Right tripod: 12 (right), 15 (right), 14 (left)
    std::vector<MotorInfo> left_tripod = {
        {13, &motor_13, true, 0.0f, 0.0f, 0.0f},   // left side
        {10, &motor_10, true, 0.0f, 0.0f, 0.0f},   // left side
        {11, &motor_11, false, 0.0f, 0.0f, 0.0f}   // right side
    };
    std::vector<MotorInfo> right_tripod = {
        {12, &motor_12, false, 0.0f, 0.0f, (float)M_PI},  // right side
        {15, &motor_15, false, 0.0f, 0.0f, (float)M_PI},  // right side
        {14, &motor_14, true, 0.0f, 0.0f, (float)M_PI}    // left side
    };

    // All motors for init/exit
    std::vector<CubemarsPi3Hat*> all_motors = {&motor_10, &motor_11, &motor_12,
                                                &motor_13, &motor_14, &motor_15};

    // Enter MIT mode for each motor with delay to allow initialization
    for (auto* motor : all_motors) {
        motor->enterMITMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Zero all motor encoders at startup

    // std::cout << "Zeroing motor encoders..." << std::endl;
    // for (auto* motor : all_motors) {
    //     motor->zeroMotor();
    //     motor->sendCommandMITMode(0.0f, 0.0f, 3.0f, 0.3f, 0.0f);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }
    //     std::cout << "Motors have been zero'd.." << std::endl;

    // Capture home positions for each tripod

    std::cout << "Capturing home positions..." << std::endl;
    CaptureHome(left_tripod);
    CaptureHome(right_tripod);
    std::cout << "Home captured." << std::endl;

    // Compute safe amplitudes for each motor based on home position

    const float A_cmd = 8.0f; // start small (rad). you can increase later.

    for (auto& info : left_tripod) {
    info.amp = ComputeSafeAmplitude(info.home_pos, A_cmd);
    std::cout << "ID " << info.id << " home=" << info.home_pos
                << " amp=" << info.amp << "\n";
    if (info.amp < 0.05f) {
  std::cout << "WARNING: ID " << info.id << " amp too small\n";
    }   
}
    for (auto& info : right_tripod) {
    info.amp = ComputeSafeAmplitude(info.home_pos, A_cmd);
    std::cout << "ID " << info.id << " home=" << info.home_pos
                << " amp=" << info.amp << "\n";
    if (info.amp < 0.05f) {
  std::cout << "WARNING: ID " << info.id << " amp too small\n";
    }   
}

    const float kp_gait = 1.0f;
    const float kd_gait = 1.0f;
    const float velocity = 3.0f;  // [rad/s]
    const float T = 10.0f; // period of gait (s)
    const auto gait_duration = std::chrono::seconds(10);
    const auto period = std::chrono::milliseconds(10);


    float phase = 0.0f;
    constexpr float TWO_PI = 2.0f * (float)M_PI;
    
    auto t0 = std::chrono::steady_clock::now();
    auto last = t0;

    std::cout << "Running tripod gait ...\n";

    while (std::chrono::steady_clock::now() - t0 < gait_duration) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last).count();
        last = now;

        phase += TWO_PI * dt / T;
        phase = std::fmod(phase, TWO_PI);

        // Tripod A (phase_off = 0)
        for (auto& info: left_tripod){
            float pos_des = info.home_pos + info.amp * std::sin(phase + info.phase_off);
            info.motor->sendCommandMITMode(pos_des, 0.0f, kp_gait, kd_gait, 0.0f);
        }

        // Tripod B (phase_off = pi)
        for (auto& info: right_tripod){
            float pos_des = info.home_pos + info.amp * std::sin(phase + info.phase_off);
            info.motor->sendCommandMITMode(pos_des, 0.0f, kp_gait, kd_gait, 0.0f);
        }

        static int dbg = 0;
        dbg++;
        if (dbg % 50 == 0) { // 50*10ms = 0.5s
        auto& a = left_tripod[0];
        auto& b = right_tripod[0];
        float p_a = a.motor->getPosition();
        float p_b = b.motor->getPosition();
        float cmd_a = a.home_pos + a.amp * std::sin(phase + a.phase_off);
        float cmd_b = b.home_pos + b.amp * std::sin(phase + b.phase_off);
        std::cout << "phase=" << phase
                    << " A: id=" << a.id << " p=" << p_a << " cmd=" << cmd_a
                    << " | B: id=" << b.id << " p=" << p_b << " cmd=" << cmd_b
                    << "\n";
        }   
        

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Returning ALL motors to home...\n";
    auto start_home = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_home < std::chrono::seconds(3)) {
    for (auto& info : left_tripod) {
        info.motor->sendCommandMITMode(info.home_pos, 0.0f, 1.0f, 1.0f, 0.0f);
    }
    for (auto& info : right_tripod) {
        info.motor->sendCommandMITMode(info.home_pos, 0.0f, 1.0f, 1.0f, 0.0f);
    }
    std::this_thread::sleep_for(period);
    }


    for (auto* motor : all_motors) {
        motor->exitMITMode();
    }

    return 0;
}

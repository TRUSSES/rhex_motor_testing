#include "cubemars_pi3hat.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <csignal>
#include <atomic>


// Motors are zero'd correctly --> Moves motors to stand pose and holds it until interrupted by Ctrl-C 
// Motors will to desired location given safe kp, kd gains, and stand angle.

namespace {

// Motor side info for direction control
struct MotorInfo {
    int id;
    CubemarsPi3Hat* motor;
    bool is_left_side;  // true = left side (positive velocity, CCW), false = right side (negative velocity, CW)
    float home_pos = 0.0f;  // Home position in radians
};

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kPMin = -12.5f;
constexpr float kPMax = 12.5f;

float SideSign(const MotorInfo& info) {
    return info.is_left_side ? 1.0f : -1.0f;
}


void CaptureHome(std::vector<MotorInfo>& tripod,
                 float kp_hold = 0.0f,
                 float kd_hold = 0.5f,
                std::chrono::milliseconds hold_time = std::chrono::milliseconds(300)) {

    const auto period = std::chrono::milliseconds(10);
    // 1) Prime feedback: send a few "do nothing" frames so getPosition() is fresh.
    for (int k = 0; k < 5; ++k) {  // 5 * 10ms = 50ms
        for (auto& info : tripod) {
        float p = info.motor->getPosition();                 // whatever is currently cached
        info.motor->sendCommandMITMode(p, 0.0f, 0.0f, 0.5f, 0.0f); // kp=0, kd=1 => damped only
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

// Stop all motors in a tripod
void StopTripod(std::vector<MotorInfo>& tripod, float kd) {
    for (auto& info : tripod) {
        info.motor->sendCommandMITMode(0.0f, 0.0f, 0.0f, kd, 0.0f);
    }
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
            info.motor->sendCommandMITMode(0.0f, velocity * SideSign(info), kp, kd, 0.0f);
        }
        // Keep other tripod stopped
        StopTripod(other_tripod, kd);
        std::this_thread::sleep_for(period);
    }
}

void MoveTripodsToOffset(std::vector<MotorInfo>& left_tripod,
                         std::vector<MotorInfo>& right_tripod,
                         float offset_rad,
                         float kp,
                         float kd,
                         std::chrono::milliseconds duration) {
    const auto period = std::chrono::milliseconds(10);
    auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start < duration) {
        for (auto& info : left_tripod) {
            float target = info.home_pos + (SideSign(info) * offset_rad);
            float clamped = std::clamp(target, kPMin, kPMax);
            info.motor->sendCommandMITMode(clamped, 0.0f, kp, kd, 0.0f);
        }
        for (auto& info : right_tripod) {
            float target = info.home_pos + (SideSign(info) * offset_rad);
            float clamped = std::clamp(target, kPMin, kPMax);
            info.motor->sendCommandMITMode(clamped, 0.0f, kp, kd, 0.0f);
        }
        std::this_thread::sleep_for(period);
    }
}

void PrimeFeedback(const std::vector<CubemarsPi3Hat*>& motors) {
    const auto period = std::chrono::milliseconds(5);
    for (int k = 0; k < 5; ++k) {
        for (auto* motor : motors) {
            float p = motor->getPosition();
            motor->sendCommandMITMode(p, 0.0f, 0.0f, 0.5f, 0.0f);
        }
        std::this_thread::sleep_for(period);
    }
}

static std::atomic<bool> g_run(true);
static void HandleSigInt(int){
    g_run = false;
}


}  // namespace

int main(int argc, char** argv) {
    std::cout.setf(std::ios::unitbuf);  // auto-flush after every << operation
    int can_bus = 4;  // Default to CAN bus 4 (JC5 connector)
    bool do_zero = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--zero") {
            do_zero = true;
        } else if (arg == "--bus" && i + 1 < argc) {
            can_bus = std::atoi(argv[++i]);
        } else {
            can_bus = std::atoi(argv[i]);
        }
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
        {13, &motor_13, true},   // left side
        {10, &motor_10, true},   // left side
        {11, &motor_11, false}   // right side
    };
    std::vector<MotorInfo> right_tripod = {
        {12, &motor_12, false},  // right side
        {15, &motor_15, false},  // right side
        {14, &motor_14, true}    // left side
    };

    // All motors for init/exit
    std::vector<CubemarsPi3Hat*> all_motors = {&motor_10, &motor_11, &motor_12,
                                                &motor_13, &motor_14, &motor_15};

    ////////// Enter MIT mode for each motor with delay to allow initialization //////////
    for (auto* motor : all_motors) motor->enterMITMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(5)); // wait for motors to enter MIT mode

    // // Immediately send "limp" commands to all motors for ~100ms
    // for (int k = 0; k < 50; ++k) {  // 10 * 10ms = 100ms
    //     for (auto* motor : all_motors) {
    //         float p = motor->getPosition();
    //         motor->sendCommandMITMode(p, 0.0f, 0.0f, 1.0f, 0.0f); // no pull, no damping, no torque
    //     }
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    // // Prime comms / wrap estimator WITHOUT position hold
    // for (int k = 0; k < 50; ++k) {
    // for (auto* m : all_motors) {
    //     m->sendCommandMITMode(0.0f, 0.0f, 0.0f, 1.0f, 0.0f);  // kp=0, damping only
    //     }
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    // // Now that feedback is fresh, read p and (optionally) latch
    // for (int k = 0; k < 50; ++k) {
    // for (auto* m : all_motors) {
    //     float p = m->getPosition();
    //     m->sendCommandMITMode(p, 0.0f, 0.0f, 1.0f, 0.0f);
    //     }
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
    std::signal(SIGINT, HandleSigInt);

    // 1) Pre-zero readout
    std::cout << "Pre-zero positions..." << std::endl;
    PrimeFeedback(all_motors);
    for (const auto& info : left_tripod) 
        std::cout << "ID " << info.id
                    << " pos=" << info.motor->getPosition() << " rad\n";
    for (const auto& info : right_tripod)
        std::cout << "ID " << info.id
                    << " pos=" << info.motor->getPosition() << " rad\n";


    // 2) Zero if requested
    if (do_zero) {
        std::cout << "Zeroing motor encoders..." << std::endl;
        for (auto* motor : all_motors) {
            motor->zeroMotor();
            // Keep passive after zeroing to avoid any commanded motion.
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait before reading post-zero positions
            std::cout << "Motors have been zero'd." << std::endl;
        }

    // 3) Post-zero readout
    std::cout << "Post-zero positions..." << std::endl;
    PrimeFeedback(all_motors);
    for (const auto& info : left_tripod)
        std::cout << "ID " << info.id
                    << " pos=" << info.motor->getPosition() << " rad\n";
    
    for (const auto& info : right_tripod)
        std::cout << "ID " << info.id
                    << " pos=" << info.motor->getPosition() << " rad\n";


    // 4) Capture home after zero
    std::cout << "Capturing home positions..." << std::endl;
    CaptureHome(left_tripod);
    CaptureHome(right_tripod);
    std::cout << "Home captured." << std::endl;

    // 5) Move to a stand pose (need to setup a ramp later for smoothness)
    const float kd = 2.0f;
    const float kp_position = 2.5f;  // Position control gain for stand pose
    const float stand_deg = -95.0f;
    const auto stand_duration = std::chrono::seconds(10);


    // Move to a stand pose relative to captured home.
    std::cout << "Moving to stand (" << stand_deg << " deg)..." << std::endl;
    MoveTripodsToOffset(left_tripod, right_tripod, stand_deg * kDegToRad,
                        kp_position, kd, stand_duration);

    std::cout << "Holding stand ... (Ctrl-C to exit)\\n";

    // Holding stand pose indefinitely
    while(g_run){
        for (auto&info : left_tripod) {
            float target = info.home_pos + (SideSign(info) * (stand_deg * kDegToRad));
            target = std::clamp(target, kPMin, kPMax);
            info.motor->sendCommandMITMode(target, 0.0f, kp_position, kd, 0.0f);
        }
        for (auto&info : right_tripod) {
            float target = info.home_pos + (SideSign(info) * (stand_deg * kDegToRad));
            target = std::clamp(target, kPMin, kPMax);
            info.motor->sendCommandMITMode(target, 0.0f, kp_position, kd, 0.0f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Exiting stand hold..." << std::endl;

    for (auto* motor : all_motors) {
        motor->exitMITMode();
    }

    return 0;
}

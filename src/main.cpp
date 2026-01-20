#include "cubemars_pi3hat.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

namespace {

// Motor side info for direction control
struct MotorInfo {
    CubemarsPi3Hat* motor;
    bool is_left_side;  // true = left side (positive velocity), false = right side (negative velocity)
};

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
        {&motor_13, true},   // left side
        {&motor_10, true},   // left side
        {&motor_11, false}   // right side
    };
    std::vector<MotorInfo> right_tripod = {
        {&motor_12, false},  // right side
        {&motor_15, false},  // right side
        {&motor_14, true}    // left side
    };

    // All motors for init/exit
    std::vector<CubemarsPi3Hat*> all_motors = {&motor_10, &motor_11, &motor_12,
                                                &motor_13, &motor_14, &motor_15};

    // Enter MIT mode for each motor with delay to allow initialization
    for (auto* motor : all_motors) {
        motor->enterMITMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Zero all motor encoders at startup
    std::cout << "Zeroing motor encoders..." << std::endl;
    for (auto* motor : all_motors) {
        motor->zeroMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const float kp = 0.0f;
    const float kd = 0.8;
    const float kp_position = 0.3f;  // For position control when returning to zero
    const float velocity = 3.0f;  // [rad/s]

    const auto return_duration = std::chrono::seconds(3);
    const auto period = std::chrono::milliseconds(10);

    // Run left tripod
    std::cout << "Left tripod forward (5s)..." << std::endl;
    RunTripod(left_tripod, right_tripod, velocity, std::chrono::seconds(5), kp, kd);

    // Return left tripod to zero
    std::cout << "Returning left tripod to zero..." << std::endl;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < return_duration) {
        for (auto& info : left_tripod) {
            info.motor->sendCommandMITMode(0.0f, 0.0f, kp_position, kd, 0.0f);
        }
        StopTripod(right_tripod, kd);
        std::this_thread::sleep_for(period);
    }

    // Run right tripod
    std::cout << "Right tripod forward (5s)..." << std::endl;
    RunTripod(right_tripod, left_tripod, velocity, std::chrono::seconds(5), kp, kd);

    // Return right tripod to zero
    std::cout << "Returning right tripod to zero..." << std::endl;
    start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < return_duration) {
        for (auto& info : right_tripod) {
            info.motor->sendCommandMITMode(0.0f, 0.0f, kp_position, kd, 0.0f);
        }
        StopTripod(left_tripod, kd);
        std::this_thread::sleep_for(period);
    }

    for (auto* motor : all_motors) {
        motor->exitMITMode();
    }

    return 0;
}

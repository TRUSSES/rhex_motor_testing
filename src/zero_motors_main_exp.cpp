#include "cubemars_pi3hat.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
namespace {

// Motor side info for direction control
struct MotorInfo {
    int id;
    CubemarsPi3Hat* motor;
    bool is_left_side;  // true = left side (positive velocity, CCW), false = right side (negative velocity, CW)
    float home_pos = 0.0f;  // Home position in radians
    float return_target = 0.0f;
};

    static inline float WrapToPi(float a) {
    a = std::fmod(a + (float)M_PI, 2.0f*(float)M_PI);
    if (a < 0) a += 2.0f*(float)M_PI;
    return a - (float)M_PI;
    }


void CaptureHome(std::vector<MotorInfo>& tripod,
                 float kp_hold = 0.8f,
                 float kd_hold = 0.5f,
                std::chrono::milliseconds hold_time = std::chrono::milliseconds(300)) {

    const auto period = std::chrono::milliseconds(10);

    // 1) Prime feedback: send a few "do nothing" frames so getPosition() is fresh.
    for (int k = 0; k < 5; ++k) {  // ~50ms
        for (auto& info : tripod) {
        float p = info.motor->getPosition();                 // whatever is currently cached
        info.motor->sendCommandMITMode(p, 0.0f, 0.0f, kd_hold, 0.0f); // kp=0 => no pull
        }
        std::this_thread::sleep_for(period);
    }

    // Read absolute poistions ONCE as a hold target
    for (auto& info: tripod) {
        float p_abs = info.motor->getPosition();
        info.return_target = p_abs;

        std::cout << "ID " << info.id << " init_pos: " << 
        p_abs << " (init_wrap " << WrapToPi(p_abs) << ")\n";
    }

    // hold current position for a short time to capture home
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < hold_time) {
        for (auto& info : tripod) {
            // Hold at whatever it currently is by using v_des=0 and a small damping.
            // We still set home_pos=0 byt with kp=0 so it won't pull to zero.
            info.motor->sendCommandMITMode(info.return_target, 0.0f, kp_hold, kd_hold, 0.0f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Now read absolute positions as home
    for (auto& info : tripod) {
        float p_abs = info.motor->getPosition();
        info.home_pos = WrapToPi(p_abs);
        std::cout << "ID " << info.id << " home_position_abs: " << 
        p_abs << "(home_wrap " << WrapToPi(p_abs) << ")\n";
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

    // Enter MIT mode for each motor with delay to allow initialization
    for (auto* motor : all_motors) {
        motor->enterMITMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // std::cout << "Testing feedback for IDs 10 and 15...\n";

    // // command tiny velocity briefly
    // for (int k = 0; k < 20; k++) { // 200ms at 10ms
    // motor_10.sendCommandMITMode(0.0f, 0.5f, 0.0f, 0.3f, 0.0f);
    // motor_15.sendCommandMITMode(0.0f, 0.5f, 0.0f, 0.3f, 0.0f);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    // // stop
    // for (int k = 0; k < 20; k++) {
    // motor_10.sendCommandMITMode(0.0f, 0.0f, 0.0f, 0.3f, 0.0f);
    // motor_15.sendCommandMITMode(0.0f, 0.0f, 0.0f, 0.3f, 0.0f);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    // // read
    // std::cout << "p10=" << motor_10.getPosition()
    //         << " (wrap " << WrapToPi(motor_10.getPosition()) << ")\n";
    // std::cout << "p15=" << motor_15.getPosition()
    //         << " (wrap " << WrapToPi(motor_15.getPosition()) << ")\n";


    // Zero all motor encoders at startup

    // std::cout << "Zeroing motor encoders..." << std::endl;
    // for (auto* motor : all_motors) {
    //     motor->zeroMotor();
    //     motor->sendCommandMITMode(0.0f, 0.0f, 3.0f, 0.3f, 0.0f);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }
    //     std::cout << "Motors have been zero'd.." << std::endl;

    std::cout << "Capturing home positions..." << std::endl;
    CaptureHome(left_tripod);
    CaptureHome(right_tripod);
    std::cout << "Home captured." << std::endl;



    const float kp = 0.0f;
    const float kd = 0.8;
    const float kp_position = 0.2f;  // For position control when returning to zero
    const float velocity = 3.0f;  // [rad/s]

    const auto return_duration = std::chrono::seconds(5);
    const auto period = std::chrono::milliseconds(10);


    ////// TRIPOD WALKING SEQUENCE //////


    // Run left tripod //

    std::cout << "Left tripod forward (5s)..." << std::endl;
    RunTripod(left_tripod, right_tripod, velocity, std::chrono::seconds(5), kp, kd);

    // Return left tripod to zero
    std::cout << "Returning left tripod to zero..." << std::endl;

    // // Break first, strong damping no position pull
    // const auto left_brake_time = std::chrono::milliseconds(500);
    // auto tb_L = std::chrono::steady_clock::now();
    // while (std::chrono::steady_clock::now() - tb_L < left_brake_time)
    // {
    //     for (auto& info : left_tripod) {
    //         float p = info.motor->getPosition();
    //         info.motor->sendCommandMITMode(p, 0.0f, 0.0f, 1.2f, 0.0f);
    //     }
    //     StopTripod(right_tripod, kd);
    //     std::this_thread::sleep_for(period);
    // }

    // Compute fixed return targets based on nearest wrap-around ONCE

    for (auto& info : left_tripod) {
        float p0 = info.motor->getPosition();
        float p0w = WrapToPi(p0);

        // desired shortest wrapped target to home
        float err = WrapToPi(info.home_pos - p0w);

        // command in the same coordinate as p0
        info.return_target = p0 + err;
        
        

        std::cout << "ID " << info.id
                << " p0=" << p0 << " (wrap " << WrapToPi(p0) << ")"
                << " home=" << info.home_pos << " (wrap " << WrapToPi(info.home_pos) << ")"
                << " target=" << info.return_target << " (wrap " << WrapToPi(info.return_target) << ")\n";
        }
    
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < return_duration) {
        for (auto& info : left_tripod) {
            // Moves legs to nearest equivalent position to home
            info.motor->sendCommandMITMode(info.return_target, 0.0f, kp_position, kd, 0.0f);
        }
        StopTripod(right_tripod, kd);
        std::this_thread::sleep_for(period);
    }

    /////////// RUN RIGHT TRIPOD /////////////

    std::cout << "Right tripod forward (5s)..." << std::endl;
    RunTripod(right_tripod, left_tripod, velocity, std::chrono::seconds(5), kp, kd);

    // Return right tripod to zero
    std::cout << "Returning right tripod to zero..." << std::endl;

    // // Break first, strong damping no position pull
    // const auto right_brake_time = std::chrono::milliseconds(500);
    // auto tb_R = std::chrono::steady_clock::now();
    // while (std::chrono::steady_clock::now() - tb_R < right_brake_time)
    // {
    //     for (auto& info : right_tripod) {
    //         float p = info.motor->getPosition();
    //         info.motor->sendCommandMITMode(p, 0.0f, 0.0f, 1.2f, 0.0f);
    //     }
    //     StopTripod(left_tripod, kd);
    //     std::this_thread::sleep_for(period);
    // }

    // Compute fixed return targets based on nearest wrap-around ONCE

    for (auto& info : right_tripod) {
        float p0 = info.motor->getPosition();
        float p0w = WrapToPi(p0);

        // desired shortest wrapped target to home
        float err = WrapToPi(info.home_pos - p0w);

        // command in the same coordinate as p0
        info.return_target = p0 + err;
        

        std::cout << "ID " << info.id
                << " p0=" << p0 << " (wrap " << WrapToPi(p0) << ")"
                << " home=" << info.home_pos << " (wrap " << WrapToPi(info.home_pos) << ")"
                << " target=" << info.return_target << " (wrap " << WrapToPi(info.return_target) << ")\n";
        }
    
    start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < return_duration) {
        for (auto& info : right_tripod) {
            // Moves legs to nearest equivalent position to home
            info.motor->sendCommandMITMode(info.return_target, 0.0f, kp_position, kd, 0.0f);

        }
        StopTripod(left_tripod, kd);
        std::this_thread::sleep_for(period);
    }

    /// EXIT SEQUENCE ///

    for (auto* motor : all_motors) {
        motor->exitMITMode();
    }

    return 0;
}

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
// Adding functuinality to have a smooth transition to stand pose and hold. 

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

const float step_angle = 2.0f * kPi; // 360 degrees per step
const auto step_duration = std::chrono::seconds(4); // 4 seconds per step | NEED TO TUNE THESE VALUES


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

// Adding trajectory-shaped that ramps from motor's actual to position to the target
float SmoothStep (float u){
    u = std::clamp(u, 0.0f, 1.0f);
    return u * u * (3.0f - 2.0f * u);  // 0-->1 with zero slope at the ends
}

// Global mode for signal handling
enum class Mode {kHoldStand, kTripodWalk, kReturnHome, kHoldHome, kExit};
static std::atomic<Mode> g_mode(Mode::kHoldStand);


// Move both tripods to an offset from home with a smooth ramp
void MoveTripodsToOffset(std::vector<MotorInfo>& left_tripod,
                         std::vector<MotorInfo>& right_tripod,
                         float offset_rad,
                         float kp_move,
                         float kd_move,
                         std::chrono::milliseconds duration,
                         float kp_hold = -1.0f,
                         float kd_hold = -1.0f) {
    const auto period = std::chrono::milliseconds(10);
    const float T = std::max(0.001f, duration.count() / 1000.0f); // duration in seconds

    // Use current position as start to avoid jump if home_pos/sign/wrap is off.

    struct RampState { float p0; float p1; };
    std::vector<RampState> L(left_tripod.size()), R(right_tripod.size());

    for (size_t i = 0; i < left_tripod.size(); ++i) {
        float p0 = left_tripod[i].motor->getPosition();
        float p1 = left_tripod[i].home_pos + (SideSign(left_tripod[i]) * offset_rad);
        L[i] = {p0 , std::clamp(p1, kPMin, kPMax)};
    }

    for (size_t i = 0; i < right_tripod.size(); ++i) {
        float p0 = right_tripod[i].motor->getPosition();
        float p1 = right_tripod[i].home_pos + (SideSign(right_tripod[i]) * offset_rad);
        R[i] = {p0 , std::clamp(p1, kPMin, kPMax)};
    }

    bool aborted = false;

    auto start = std::chrono::steady_clock::now();
    while(true){

        Mode m = g_mode.load();
        // if (m == Mode::kExit || m == Mode::kReturnHome) {aborted = true; break;}
        if (m == Mode::kExit || m == Mode::kReturnHome) {
            aborted = true;
            break;
        }

        float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        float u = t / T;
        if (u >= 1.0f) break; // <-- move phase ends right here

        float s = SmoothStep(u);

        for (size_t i = 0; i < left_tripod.size(); ++i){
            float p_cmd = L[i].p0 + s * (L[i].p1 - L[i].p0);
            left_tripod[i].motor -> sendCommandMITMode(p_cmd, 0.0f, kp_move, kd_move, 0.0f);
        }

        for (size_t i = 0; i < right_tripod.size(); ++i){
            float p_cmd = R[i].p0 + s * (R[i].p1 - R[i].p0);
            right_tripod[i].motor -> sendCommandMITMode(p_cmd, 0.0f, kp_move, kd_move, 0.0f);
        }
        std::this_thread::sleep_for(period); // fixed-rate command streaming
    }
    if (aborted) return;

    // Send final target once more (still with move gains) so we land at exactly p1.
    for (size_t i = 0; i < left_tripod.size(); ++i){
        left_tripod[i].motor -> sendCommandMITMode(L[i].p1, 0.0f, kp_move, kd_move, 0.0f);
    }
    for (size_t i = 0; i < right_tripod.size(); ++i){
        right_tripod[i].motor -> sendCommandMITMode(R[i].p1, 0.0f, kp_move, kd_move, 0.0f);
    }

    // Maybe: immediately switch to hold gains
    if (kp_hold >= 0.0f && kd_hold >= 0.0f) {
        for (size_t i = 0; i < left_tripod.size(); ++i){
            left_tripod[i].motor -> sendCommandMITMode(L[i].p1, 0.0f, kp_hold, kd_hold, 0.0f);
        }
        for (size_t i = 0; i < right_tripod.size(); ++i){
            right_tripod[i].motor -> sendCommandMITMode(R[i].p1, 0.0f, kp_hold, kd_hold, 0.0f);
        }
    }

}

void AdvanceTripodPhase(std::vector<MotorInfo>& active,
                        std::vector<MotorInfo>& support,
                        float stand_offset_rad,
                        float phase_start,
                        float phase_end,
                        float kp_tripod, float kd_tripod,
                        float kp_hold, float kd_hold,
                        std::chrono::milliseconds duration) {

    const auto period = std::chrono::milliseconds(10);
    const float T = std::max(0.001f, duration.count() / 1000.0f); // duration in seconds

    auto start = std::chrono::steady_clock::now();
    while (true) {
        Mode m = g_mode.load();
        if (m == Mode::kExit || m == Mode::kReturnHome) return; //allow state change to interrupt

        float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        float u = t / T;
        if (u >= 1.0f) u = 1.0f; //clamp to end

        float s = SmoothStep(u);
        float phase = phase_start + s * (phase_end - phase_start);

        // Support Tripod holds stand 
        for (auto& info: support){
            float stand_target = info.home_pos + (SideSign(info) * stand_offset_rad);
            stand_target = std::clamp(stand_target, kPMin, kPMax);
            info.motor -> sendCommandMITMode(stand_target, 0.0f, kp_hold, kd_hold, 0.0f);
        }

        // Active Tripod moves through phase
        for(auto&info: active){
            float stand_target = info.home_pos + (SideSign(info) * stand_offset_rad);
            stand_target = std::clamp(stand_target, kPMin, kPMax);
            info.motor -> sendCommandMITMode(stand_target, 0.0f, kp_tripod, kd_tripod, 0.0f);
        }

        if (u >= 1.0f) break;
        std::this_thread::sleep_for(period); // fixed-rate command streaming
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


static void HandleSigInt(int){
    Mode m = g_mode.load();
    if (m == Mode::kHoldStand)          g_mode.store(Mode::kReturnHome);
    else if (m == Mode::kHoldHome)      g_mode.store(Mode::kExit);
    else                                g_mode.store(Mode::kExit);
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
    const float kd_move = 3.5f; // Damping gain for moving from home to stand
    const float kp_move = 6.0f; // Position gain for moving from home to stand

    const float kd_hold = 1.5f;  // Damping firmer once at pose
    const float kp_hold = 5.0f; // Position control gain for stand pose

    const float kd_sit = 5.0f; // Damping gain for returning to home
    const float kp_sit = 1.0f; // Position gain for returning to home

    const float kd_tripod = 4.0f; // Damping gain for tripod gait
    const float kp_tripod = 2.0f; // Position gain for tripod gait

    const float stand_deg = -110.0f;
    const auto stand_duration = std::chrono::seconds(10); // desired time to reach to pose

    //////////// Beginning main state machine loop //////////

    /////////////////////////////////////////////
    ///////////           POSE           ////////
    /////////////////////////////////////////////

    g_mode.store(Mode::kHoldStand);

    std::cout << "Moving to stand (" << stand_deg << " deg)...\n";
    MoveTripodsToOffset(left_tripod, right_tripod, stand_deg * kDegToRad,
                    kp_move, kd_move, stand_duration, kp_hold, kd_hold);
    std::cout << "Holding stand ... (Ctrl-C to return home)\n";



    while(g_mode.load() != Mode::kExit){
        Mode m = g_mode.load();

        if (m == Mode::kHoldStand){
            // Holding stand pose indefinitely
            for (auto&info : left_tripod) {
                float target = info.home_pos + (SideSign(info) * (stand_deg * kDegToRad));
                target = std::clamp(target, kPMin, kPMax);
                info.motor->sendCommandMITMode(target, 0.0f, kp_hold, kd_hold, 0.0f);
            }
            for (auto&info : right_tripod) {
                float target = info.home_pos + (SideSign(info) * (stand_deg * kDegToRad));
                target = std::clamp(target, kPMin, kPMax);
                info.motor->sendCommandMITMode(target, 0.0f, kp_hold, kd_hold, 0.0f);
            }
            // optional more responsive ctrl-c:
            for (int k = 0; k < 10 && g_mode.load() == Mode::kHoldStand; ++k) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        else if (m == Mode::kReturnHome){
            /////////////////////////////////////////////
            ///////////           HOME           ////////
            /////////////////////////////////////////////

            std::cout<<"Returning to home ...\n";

            MoveTripodsToOffset(left_tripod, right_tripod,
                0.0f, // <-- offset_rad = 0, means target is home_pos
                kp_move, kd_move,  
                std::chrono::seconds(5),
                kp_hold, kd_hold);

            std::cout << "At home position. Holding ... (Ctrl-C to exit)\n";
            g_mode.store(Mode::kHoldHome);
            continue;
        }
        else if (m == Mode::kHoldHome){
                // Safety: in case we missed the first move command, keep commanding home
            
                for (auto&info : left_tripod) {
                    float target = info.home_pos; // back to home
                    target = std::clamp(target, kPMin, kPMax);
                    info.motor->sendCommandMITMode(target, 0.0f, kp_sit, kd_sit, 0.0f);
                }
                for (auto&info : right_tripod) {
                    float target = info.home_pos; // back to home
                    target = std::clamp(target, kPMin, kPMax);
                    info.motor->sendCommandMITMode(target, 0.0f, kp_sit, kd_sit, 0.0f);
                }
                for (int k = 0; k < 10 && g_mode.load() == Mode::kHoldHome; ++k) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                } 
                
                std::cout << "Exiting stand hold..." << std::endl;

            }
        else if (m == Mode::kTripodWalk){
            /////////////////////////////////////////////
            ///////////        TRIPOD WALK       ////////
            ///////////////////////////////////////////// 

            // A = left_tripod swings, B supports, then swap

            std::cout << "Starting tripod walking gait..." << std::endl;
            AdvanceTripodPhase(left_tripod, right_tripod,
                               stand_deg * kDegToRad,
                               0.0f, step_angle,
                               kp_tripod, kd_tripod,
                               kp_hold, kd_hold,
                               step_duration);
        }
            if (g_mode.load() == Mode::kTripodWalk) continue;

            AdvanceTripodPhase(left_tripod, right_tripod,
                               stand_deg * kDegToRad,
                               0.0f, step_angle,
                               kp_tripod, kd_tripod,
                               kp_hold, kd_hold,
                               step_duration);
    }
        

    for (auto* motor : all_motors) {
        motor->exitMITMode();
    }

    return 0;
}

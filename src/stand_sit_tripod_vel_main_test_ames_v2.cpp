#include "cubemars_pi3hat.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <csignal>


// Stand sit tripod velocity control test with Ames testing gains.
// Includes wrap around logic for multi-turn rotation.
// Implementing a simple tripod gait with velocity control, mimicing a beuhler clock.

// Tripod gait planner (phase) with velocity execution to allow multi-turn rotation.

namespace {

struct UnWrapState{
    bool init = false;
    float prev_wrapped = 0.0f; // in [-pi, pi]
    float unwrapped = 0.0f; // continous
};

struct MotorInfo {
    int id;
    CubemarsPi3Hat* motor;
    bool is_left_side;
    float home_pos = 0.0f;
    float p_wrap = 0.0f;
    float p_unwrap = 0.0f;
    float hold_unwrapped = 0.0f;
    bool has_init = false;
    float hold_pos = 0.0f;
    UnWrapState unwrap;
    float p0_unwrapped = 0.0f;

};

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kPosWrap = 12.5f;

constexpr float kPMin = -12.5f;
constexpr float kPMax = 12.5f;
constexpr float kVelMax = 10.0f;  // safety clamp for velocity commands Actually +/-50

const float step_angle = 2.0f * kPi;  // one full rotation per phase
const auto step_duration = std::chrono::seconds(5); // high = slower?

static float g_phase_left = 0.0f;
static float g_phase_right = kPi;
static bool g_debug_tripod = true;

float SideSign(const MotorInfo& info) {
    return info.is_left_side ? -1.0f : 1.0f;
}

static inline float RadToDeg(float radians) {
    return radians * kRadToDeg;
}

float SmoothStep(float u) {
    u = std::clamp(u, 0.0f, 1.0f);
    return u * u * (3.0f - 2.0f * u);
}

float SmoothStepDeriv(float u) {
    u = std::clamp(u, 0.0f, 1.0f);
    return 6.0f * u * (1.0f - u);
}

static std::atomic<bool> g_sigint(false);


// Wrap around logic

static inline float WrapCentered(float x, float half_range) {
    const float period = 2.0f * half_range;
    x = std::fmod(x + half_range, period);
    if (x < 0.0f) x += period;
    return x - half_range;  // in (-half_range, +half_range]
}

// Safe Wrap jump treshold
static inline float UpdateUnwrapRange(UnWrapState& st, float meas_rad, float half_range) {
    // meas_rad is already in [-half_range, +half_range] from the motor
    if (!st.init) {
        st.init = true;
        st.prev_wrapped = meas_rad;
        st.unwrapped = meas_rad;
        return st.unwrapped;
    }
    const float d = WrapCentered(meas_rad - st.prev_wrapped, half_range); // shortest delta on period=2*half_range
    st.unwrapped += d;
    st.prev_wrapped = meas_rad;
    return st.unwrapped;
}

// phase logs/state vars
static inline float WrapPhase(float phase){
    float out = std::fmod(phase, kTwoPi);
    if (out < 0.0f) out += kTwoPi;
    return out;
}



// State commands
enum class Mode {kHoldStand, kTripodWalk, kReturnHome, kHoldHome, kExit};
static std::atomic<Mode> g_mode(Mode::kHoldStand);

static std::atomic<bool> g_input_run(true);

void InputThread() {
    std::string cmd;
    while (g_input_run.load() && std::getline(std::cin, cmd)) {
        cmd.erase(0, cmd.find_first_not_of(" \t\r\n"));
        cmd.erase(cmd.find_last_not_of(" \t\r\n") + 1);

        if (cmd == "stand")         g_mode.store(Mode::kHoldStand);
        else if (cmd == "tripod")   g_mode.store(Mode::kTripodWalk);
        else if (cmd == "home")     g_mode.store(Mode::kReturnHome);
        else if (cmd == "holdhome") g_mode.store(Mode::kHoldHome);
        else if (cmd == "exit")     g_mode.store(Mode::kExit);
        else {
            std::cout << "Commands: stand | tripod | home | holdhome | exit\n";
        }
    }
}
void SigintHandler(int) {
    g_mode.store(Mode::kExit);
    g_input_run.store(false);
}


// Hold position to zero the motors
void PrimeFeedback(const std::vector<CubemarsPi3Hat*>& motors) {
    const auto period = std::chrono::milliseconds(5);
    for (int k = 0; k < 5; ++k) {
        for (auto* motor : motors) {
            float p = motor->getPosition();
            motor->sendCommandMITMode(p, 0.0f, 0.0f, 0.6f, 0.0f);
        }
        std::this_thread::sleep_for(period);
    }
}

void CaptureHome(std::vector<MotorInfo>& tripod,
                 float kp_hold = 0.0f,
                 float kd_hold = 0.5f,
                 std::chrono::milliseconds hold_time = std::chrono::milliseconds(300)) {
    const auto period = std::chrono::milliseconds(10);
    for (int k = 0; k < 5; ++k) {
        for (auto& info : tripod) {
            float p = info.motor->getPosition();
            info.motor->sendCommandMITMode(p, 0.0f, 0.0f, 0.5f, 0.0f);
        }
        std::this_thread::sleep_for(period);
    }

    for (auto& info : tripod) {
        info.home_pos = info.motor->getPosition();
        std::cout << "ID " << info.id << " Initial read position: "
                  << info.home_pos << " rad" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < hold_time) {
        for (auto& info : tripod) {
            info.motor->sendCommandMITMode(info.home_pos, 0.0f, kp_hold, kd_hold, 0.0f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (auto& info : tripod) {
        float pos = info.motor->getPosition();
        info.home_pos = pos;
        std::cout << "ID " << info.id << " Captured home position: " << pos
                  << " rad" << std::endl;
    }
}

// Return to home
void MoveTripodsToOffset(std::vector<MotorInfo>& left_tripod,
                         std::vector<MotorInfo>& right_tripod,
                         float offset_rad,
                         float kp_move,
                         float kd_move,
                         std::chrono::milliseconds duration,
                         float kp_hold = -1.0f,
                         float kd_hold = -1.0f) {
    const auto period = std::chrono::milliseconds(10);
    const float T = std::max(0.001f, duration.count() / 1000.0f);

    struct RampState { float p0; float p1; };
    std::vector<RampState> L(left_tripod.size()), R(right_tripod.size());

    for (size_t i = 0; i < left_tripod.size(); ++i) {
        float p0 = left_tripod[i].motor->getPosition();
        float p1 = left_tripod[i].home_pos + (SideSign(left_tripod[i]) * offset_rad);
        L[i] = {p0, std::clamp(p1, kPMin, kPMax)};
    }
    for (size_t i = 0; i < right_tripod.size(); ++i) {
        float p0 = right_tripod[i].motor->getPosition();
        float p1 = right_tripod[i].home_pos + (SideSign(right_tripod[i]) * offset_rad);
        R[i] = {p0, std::clamp(p1, kPMin, kPMax)};
    }

    bool aborted = false;
    auto start = std::chrono::steady_clock::now();
    while (true) {
        Mode m = g_mode.load();
        if (m == Mode::kExit || m == Mode::kReturnHome) {
            aborted = true;
            break;
        }

        float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        float u = t / T;
        if (u >= 1.0f) break;

        float s = SmoothStep(u);
        for (size_t i = 0; i < left_tripod.size(); ++i) {
            float p_cmd = L[i].p0 + s * (L[i].p1 - L[i].p0);
            left_tripod[i].motor->sendCommandMITMode(p_cmd, 0.0f, kp_move, kd_move, 0.0f);
        }
        for (size_t i = 0; i < right_tripod.size(); ++i) {
            float p_cmd = R[i].p0 + s * (R[i].p1 - R[i].p0);
            right_tripod[i].motor->sendCommandMITMode(p_cmd, 0.0f, kp_move, kd_move, 0.0f);
        }
        std::this_thread::sleep_for(period);
    }
    if (aborted) return;

    for (size_t i = 0; i < left_tripod.size(); ++i) {
        left_tripod[i].motor->sendCommandMITMode(L[i].p1, 0.0f, kp_move, kd_move, 0.0f);
    }
    for (size_t i = 0; i < right_tripod.size(); ++i) {
        right_tripod[i].motor->sendCommandMITMode(R[i].p1, 0.0f, kp_move, kd_move, 0.0f);
    }

    if (kp_hold >= 0.0f && kd_hold >= 0.0f) {
        for (size_t i = 0; i < left_tripod.size(); ++i) {
            left_tripod[i].motor->sendCommandMITMode(L[i].p1, 0.0f, kp_hold, kd_hold, 0.0f);
        }
        for (size_t i = 0; i < right_tripod.size(); ++i) {
            right_tripod[i].motor->sendCommandMITMode(R[i].p1, 0.0f, kp_hold, kd_hold, 0.0f);
        }
    }
}

// Tripod velocity control
void AdvanceTripodPhaseVelocity(std::vector<MotorInfo>& active,
                                std::vector<MotorInfo>& support,
                                float stand_offset_rad,
                                float phase_start,
                                float phase_end,
                                float kd_tripod,
                                float kp_hold,
                                float kd_hold,
                                std::chrono::milliseconds duration) {
    // debug
    auto last_print = std::chrono::steady_clock::now();

    // Control loop at 10ms = 100hz
    const auto period = std::chrono::milliseconds(10);
    const float T = std::max(0.001f, duration.count() / 1000.0f);

    // 1) Capture a fixed hold state for stance
    for (auto& s : support) {
        s.hold_pos = std::clamp(s.motor->getPosition(), kPMin, kPMax);
    }

    // 2) Init wrap + capture each motor's cont. pos
    struct ActiveStart { float p0_unwrapped = 0.0f; };
    std::vector<ActiveStart> astart(active.size());

    for (size_t i = 0; i < active.size(); ++i) {
        // reset unwrap so each phase is relative to phase start
        active[i].unwrap = UnWrapState{};
        astart[i].p0_unwrapped =
            UpdateUnwrapRange(active[i].unwrap,
                              active[i].motor->getPosition(),
                              kPosWrap);
    }

    // 3) Phase profile only needs the delta
    const float dphase = (phase_end - phase_start);

    // 4) Start loop phase
    auto start = std::chrono::steady_clock::now();
    while (true) {
        Mode m = g_mode.load();
        // Exit command
        if (m == Mode::kExit || m == Mode::kReturnHome) return;

        float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        float u = t / T;
        if (u >= 1.0f) u = 1.0f;

        const float s = SmoothStep(u);
        const float sdot = SmoothStepDeriv(u) / T;

        // Progress within this step
        const float phase_prog = s * dphase;
        const float phase_dot = dphase * sdot;

        // ---- DEBUG GATE (prints once per 100ms) ----
        const auto now = std::chrono::steady_clock::now();
        const bool do_print = g_debug_tripod && (now - last_print >= std::chrono::milliseconds(100));

        // --- SUPPORT: hold stance targets (fixed) ---
        int clampS = 0;
        for (auto& sleg : support) {
            float cmd = std::clamp(sleg.hold_pos, kPMin, kPMax);
            if (cmd != sleg.hold_pos) clampS++;
            sleg.motor->sendCommandMITMode(cmd, 0.0f, kp_hold, kd_hold, 0.0f);
        }

        // --- ACTIVE: velocity execution with unwrap-based “go to +360deg” target ---
        for (size_t i = 0; i < active.size(); i++) {
            auto& aleg = active[i];
            // desired cont pos for this phase
            const float sign = SideSign(aleg);

            // continuous measured position (correct unwrap on ±12.5)
            const float p_meas = aleg.motor->getPosition();
            const float p_meas_u = UpdateUnwrapRange(aleg.unwrap, p_meas, kPosWrap);

            // desired continuous position
            const float p_des_u = astart[i].p0_unwrapped + sign * phase_prog;

            // leg-phase coords for wrapped error
            const float e_phase = p_des_u - p_meas_u;
            const float kp_u = 2.0f;
            float v_cmd = sign * phase_dot + kp_u * e_phase;

            const float v_meas_motor = aleg.motor->getVelocity();   // only if available
            v_cmd = std::clamp(v_cmd, -kVelMax, kVelMax);

            if (do_print) {
                std::cout
                    << "  [A] ID " << aleg.id
                    << " p=" << p_meas
                    << " pu=" << p_meas_u
                    << " e=" << e_phase
                    << " vcmd=" << v_cmd
                    << " vmeas=" << v_meas_motor
                    << "\n";
            }

            const float p_safe = std::clamp(p_meas, kPMin, kPMax);
            aleg.motor->sendCommandMITMode(p_safe, v_cmd, 0.0f, kd_tripod, 0.0f);
        }

        if (g_debug_tripod) {
            auto now2 = std::chrono::steady_clock::now();
            if (now2 - last_print >= std::chrono::milliseconds(100)) {
                std::cout << "[TripodPhaseVel] u=" << u
                          << " phase_prog=" << phase_prog
                          << " vel_ff=" << phase_dot
                          << " clampS=" << clampS
                          << "\n";
                last_print = now2;
            }
        }

        if (u >= 1.0f) break;
        std::this_thread::sleep_for(period);
    }

    // Stop active tripod but safe pos
    for (auto& aleg : active) {
        const float p_safe = std::clamp(aleg.motor->getPosition(), kPMin, kPMax);
        aleg.motor->sendCommandMITMode(p_safe, 0.0f, 0.0f, kd_tripod, 0.0f);
    }
}

// ---------------- Two-stroke (Buehler/RHex-style) clock ----------------
// Parameters:
//   tc    : total cycle period [s]
//   ts    : duration of slow (stance) portion [s]   (0 < ts < tc)
//   phi_s : angular sweep during slow portion [rad] (0 < phi_s < 2pi)
// The remaining (2pi-phi_s) is completed during the fast (swing) portion.

struct TwoStrokeClock{
    float tc; // total period of a single leg
    float ts; // duration of slow leg swing
    float phi_s; // slow phase sweep (rad)
    float phi_o; // offset between legs (rad)

    void eval(float t, float& phi_des_u, float& omega_des) const {
        // t > 0
        // Continous phase phi_u, and its derivative omega. 
        // phi_u
        const float k = std::floor(t/tc);
        const float t_cycle = t - k * tc;

        if (t_cycle < ts){ // slow phase
            omega_des = phi_s / std::max(1e-6f, ts);
            phi_des_u = phi_o + kTwoPi * k + omega_des * t_cycle;
        } else { // fast phase
            const float tf = std::max(1e-6f, tc-ts);
            const float omega_fast = (kTwoPi - phi_s) / (tc - tf);
            omega_des = omega_fast;
            phi_des_u = phi_o + kTwoPi * k + phi_s + omega_fast * (t_cycle - ts);
        }
    }
};

// Run continous two-stroke tripod gait using velocity commands + unwrap phase tracking.
// Tripod B is time-shifted by tc/2 (180 deg out of phase in time).
void RunTwoStrokeTripod(std::vector<MotorInfo>& tripodA,
                        std::vector<MotorInfo>& tripodB,
                        const TwoStrokeClock& clock,
                        float kd_tripod,
                        float k_phase_vel,
                        float vel_limit,
                        bool debug_print = true){
    // Control loop at 10ms = 100hz
    const auto period = std::chrono::milliseconds(10);

    // init gait + capture starting position for each motor ONCE.
    for (auto& leg : tripodA) {
        leg.unwrap = UnWrapState{};
        leg.p0_unwrapped = UpdateUnwrapRange(leg.unwrap, leg.motor->getPosition(), kPosWrap);
    }
    for (auto& leg : tripodB) {
        leg.unwrap = UnWrapState{};
        leg.p0_unwrapped = UpdateUnwrapRange(leg.unwrap, leg.motor->getPosition(), kPosWrap);
    }

    // TODO: implement full two-stroke gait loop.
    static_cast<void>(clock);
    static_cast<void>(kd_tripod);
    static_cast<void>(k_phase_vel);
    static_cast<void>(vel_limit);
    static_cast<void>(debug_print);
    static_cast<void>(period);
}

}  // namespace

int main(int argc, char** argv) {
    std::cout.setf(std::ios::unitbuf);
    int can_bus = 4;
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

    mjbots::pi3hat::Pi3Hat::Configuration config;
    config.can[4].slow_bitrate = 1000000;
    config.can[4].fdcan_frame = false;
    config.can[4].bitrate_switch = false;
    config.can[0].slow_bitrate = 0;
    config.can[1].slow_bitrate = 0;
    config.can[2].slow_bitrate = 0;
    config.can[3].slow_bitrate = 0;

    mjbots::pi3hat::Pi3Hat pi3hat(config);

    CubemarsPi3Hat motor_10(10, can_bus, &pi3hat);
    CubemarsPi3Hat motor_11(11, can_bus, &pi3hat);
    CubemarsPi3Hat motor_12(12, can_bus, &pi3hat);
    CubemarsPi3Hat motor_13(13, can_bus, &pi3hat);
    CubemarsPi3Hat motor_14(14, can_bus, &pi3hat);
    CubemarsPi3Hat motor_15(15, can_bus, &pi3hat);

    std::vector<MotorInfo> left_tripod = {
        {13, &motor_13, true},
        {10, &motor_10, true},
        {11, &motor_11, false}
    };
    std::vector<MotorInfo> right_tripod = {
        {12, &motor_12, false},
        {15, &motor_15, false},
        {14, &motor_14, true}
    };

    std::vector<CubemarsPi3Hat*> all_motors = {
        &motor_10, &motor_11, &motor_12, &motor_13, &motor_14, &motor_15
    };

    for (auto* motor : all_motors) motor->enterMITMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    std::thread input_thr(InputThread);
    input_thr.detach();

    std::cout << "Pre-zero positions..." << std::endl;
    PrimeFeedback(all_motors);
    for (const auto& info : left_tripod)
        std::cout << "ID " << info.id << " pos=" << info.motor->getPosition() << " rad\n";
    for (const auto& info : right_tripod)
        std::cout << "ID " << info.id << " pos=" << info.motor->getPosition() << " rad\n";

    if (do_zero) {
        std::cout << "Zeroing motor encoders..." << std::endl;
        for (auto* motor : all_motors) {
            motor->zeroMotor();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Motors have been zero'd." << std::endl;
    }

    std::cout << "Post-zero positions..." << std::endl;
    PrimeFeedback(all_motors);
    for (const auto& info : left_tripod)
        std::cout << "ID " << info.id << " pos=" << info.motor->getPosition() << " rad\n";
    for (const auto& info : right_tripod)
        std::cout << "ID " << info.id << " pos=" << info.motor->getPosition() << " rad\n";

    std::cout << "Capturing home positions..." << std::endl;
    CaptureHome(left_tripod);
    CaptureHome(right_tripod);
    std::cout << "Home captured." << std::endl;

    const float kd_move = 8.5f;
    const float kp_move = 15.5f; 

    const float kd_hold = 12.5f;  // Damping firmer once at pose
    const float kp_hold = 15.5f; // Position control gain for stand pose

    const float kd_sit = 16.5f;
    const float kp_sit = 0.25f;

    const float kd_tripod = 8.0f; // Damping gain for tripod gait
    // const float kp_tripod = 4.5f; // Position gain for tripod gait

    const float stand_deg = 116.0f; //STAND OFFSET

    std::signal(SIGINT, SigintHandler);

    /////////////////  STATE MACHINE /////////////////

    g_mode.store(Mode::kHoldHome);
    std::cout << "Ready. Type: stand | tripod | home | holdhome | exit\n";

    while (g_mode.load() != Mode::kExit) {
        Mode m = g_mode.load();

        if (m == Mode::kHoldStand) {
            for (auto& info : left_tripod) {
                float target = info.home_pos + (SideSign(info) * (stand_deg * kDegToRad));
                target = std::clamp(target, kPMin, kPMax);
                info.motor->sendCommandMITMode(target, 0.0f, kp_hold, kd_hold, 0.0f);
            }
            for (auto& info : right_tripod) {
                float target = info.home_pos + (SideSign(info) * (stand_deg * kDegToRad));
                target = std::clamp(target, kPMin, kPMax);
                info.motor->sendCommandMITMode(target, 0.0f, kp_hold, kd_hold, 0.0f);
            }
            for (int k = 0; k < 10 && g_mode.load() == Mode::kHoldStand; ++k) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } else if (m == Mode::kReturnHome) {
            std::cout << "Returning to home ...\n";
            MoveTripodsToOffset(left_tripod, right_tripod,
                                0.0f,
                                kp_move, kd_move,
                                std::chrono::seconds(5),
                                kp_hold, kd_hold);
            std::cout << "At home position. Holding ... (Ctrl-C to exit)\n";
            g_mode.store(Mode::kHoldHome);
            continue;
        } else if (m == Mode::kHoldHome) {
            for (auto& info : left_tripod) {
                float target = std::clamp(info.home_pos, kPMin, kPMax);
                info.motor->sendCommandMITMode(target, 0.0f, kp_sit, kd_sit, 0.0f);
            }
            for (auto& info : right_tripod) {
                float target = std::clamp(info.home_pos, kPMin, kPMax);
                info.motor->sendCommandMITMode(target, 0.0f, kp_sit, kd_sit, 0.0f);
            }
            for (int k = 0; k < 10 && g_mode.load() == Mode::kHoldHome; ++k) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } else if (m == Mode::kTripodWalk) {
            std::cout << "Starting tripod walking gait (velocity mode)..." << std::endl;
            while (g_mode.load() == Mode::kTripodWalk) {
                if (g_debug_tripod) {
                    std::cout << "\n=== LEFT swing phase_start=" << g_phase_left
                              << " (" << RadToDeg(g_phase_left) << " deg)"
                              << " phase_end=" << g_phase_left + step_angle
                              << " (" << RadToDeg(g_phase_left + step_angle) << " deg)"
                              << " stand=" << stand_deg << " deg ===\n";
                }

                AdvanceTripodPhaseVelocity(left_tripod, right_tripod,
                                           stand_deg * kDegToRad,
                                           g_phase_left, g_phase_left + step_angle,
                                           kd_tripod,
                                           kp_hold, kd_hold,
                                           step_duration);
                g_phase_left = WrapPhase(g_phase_left + step_angle);
                // g_phase_left += step_angle;

                if (g_mode.load() != Mode::kTripodWalk) break;

                if (g_debug_tripod) {
                    std::cout << "\n=== RIGHT swing phase_start=" << g_phase_right
                              << " (" << RadToDeg(g_phase_right) << " deg)"
                              << " phase_end=" << g_phase_right + step_angle
                              << " (" << RadToDeg(g_phase_right + step_angle) << " deg)"
                              << " stand=" << stand_deg << " deg ===\n";
                }

                AdvanceTripodPhaseVelocity(right_tripod, left_tripod,
                                           stand_deg * kDegToRad,
                                           g_phase_right, g_phase_right + step_angle,
                                           kd_tripod,
                                           kp_hold, kd_hold,
                                           step_duration);
                g_phase_right = WrapPhase(g_phase_right + step_angle);
                // g_phase_right += step_angle;
            }
        }
    }

    g_input_run.store(false);
    if (input_thr.joinable()) input_thr.join();

    std::cout << "Exiting ...." << std::endl;
    for (auto* motor : all_motors) {
        motor->exitMITMode();
    }

    return 0;
}

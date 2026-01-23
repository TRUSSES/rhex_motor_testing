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
    float hold_pos = 0.0f; // position to hold
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

void SpinTripodVelocity(std::vector<MotorInfo>& tripod,
                        float omega_rad_s,
                        std::chrono::milliseconds duration,
                        float kd) {
  auto start = std::chrono::steady_clock::now();
  const auto period = std::chrono::milliseconds(10);

  while (std::chrono::steady_clock::now() - start < duration) {
    for (auto& info : tripod) {
      float dir = info.is_left_side ? -1.0f : 1.0f;

      // keep pos_des equal to current pos (harmless, but avoids any internal surprises)
      float p = info.motor->getPosition();

      // kp=0 => position term off, pure velocity+damping behavior
      info.motor->sendCommandMITMode(p, dir * omega_rad_s, 0.0f, kd, 0.0f);
    }
    std::this_thread::sleep_for(period);
  }
}


static void HoldTripodAtCurrent(std::vector<MotorInfo>& tripod,
                                std::chrono::milliseconds duration,
                                float kp_hold,
                                float kd_hold) {
  // capture hold targets once
  std::vector<float> hold_pos(tripod.size());
  for (size_t i = 0; i < tripod.size(); ++i) {
    hold_pos[i] = tripod[i].motor->getPosition();
  }

  auto start = std::chrono::steady_clock::now();
  const auto period = std::chrono::milliseconds(10);

  while (std::chrono::steady_clock::now() - start < duration) {
    for (size_t i = 0; i < tripod.size(); ++i) {
      tripod[i].motor->sendCommandMITMode(hold_pos[i], 0.0f, kp_hold, kd_hold, 0.0f);
    }
    std::this_thread::sleep_for(period);
  }
}

static void MoveTripodByVelocity(std::vector<MotorInfo>& tripod_move,
                                 std::vector<MotorInfo>& tripod_hold,
                                 float omega_rad_s,
                                 std::chrono::milliseconds duration,
                                 float kd_move,
                                 float kp_hold,
                                 float kd_hold) {
  constexpr float P_MIN = -12.5f;
  constexpr float P_MAX =  12.5f;
  constexpr float margin = 0.25f;
  const float P_MIN_SAFE = P_MIN + margin;
  const float P_MAX_SAFE = P_MAX - margin;

  const float dt_s = std::chrono::duration<float>(duration).count();
  const float delta_cmd = omega_rad_s * dt_s;  // expected magnitude

  // capture start positions for move + hold
  std::vector<float> p0_move(tripod_move.size());
  for (size_t i = 0; i < tripod_move.size(); ++i) {
    p0_move[i] = tripod_move[i].motor->getPosition();
  }

  std::vector<float> hold_pos(tripod_hold.size());
  for (size_t i = 0; i < tripod_hold.size(); ++i) {
    hold_pos[i] = tripod_hold[i].motor->getPosition();
  }

  // pre-check clamp feasibility
  for (size_t i = 0; i < tripod_move.size(); ++i) {
    const auto& info = tripod_move[i];
    float dir = info.is_left_side ? -1.0f : 1.0f;
    float target = p0_move[i] + dir * delta_cmd;
    if (target < P_MIN_SAFE || target > P_MAX_SAFE) {
      std::cout << "WARNING: id=" << info.id
                << " target=" << target
                << " exceeds safe range [" << P_MIN_SAFE << ", " << P_MAX_SAFE << "]\n";
    }
  }

  auto start = std::chrono::steady_clock::now();
  const auto period = std::chrono::milliseconds(10);
  int tick = 0;

  while (std::chrono::steady_clock::now() - start < duration) {
    // moving tripod: velocity command
    for (auto& info : tripod_move) {
      float dir = info.is_left_side ? -1.0f : 1.0f;
      float p = info.motor->getPosition();  // keep pos_des "reasonable"
      info.motor->sendCommandMITMode(p, dir * omega_rad_s, 0.0f, kd_move, 0.0f);
    }

    // holding tripod: hold at captured positions
    for (size_t i = 0; i < tripod_hold.size(); ++i) {
      tripod_hold[i].motor->sendCommandMITMode(hold_pos[i], 0.0f, kp_hold, kd_hold, 0.0f);
    }

    // debug every ~0.2s
    tick++;
    if (tick % 20 == 0) {
      float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
      auto& a = tripod_move[0];
      float p = a.motor->getPosition();
      float v = a.motor->getVelocity();
      std::cout << "  t=" << t
                << " move_id=" << a.id
                << " p=" << p
                << " v=" << v
                << " v_cmd=" << (a.is_left_side ? -omega_rad_s : omega_rad_s)
                << "\n";
    }

    std::this_thread::sleep_for(period);
  }

  // post summary: expected vs actual
  std::cout << "Move summary (expected |Δ|=" << delta_cmd << " rad)\n";
  for (size_t i = 0; i < tripod_move.size(); ++i) {
    auto& info = tripod_move[i];
    float dir = info.is_left_side ? -1.0f : 1.0f;
    float p1 = info.motor->getPosition();
    float actual = p1 - p0_move[i];
    float expected = dir * delta_cmd;
    std::cout << "  id=" << info.id
              << " p0=" << p0_move[i]
              << " p1=" << p1
              << " actualΔ=" << actual
              << " expectedΔ=" << expected
              << "\n";
  }
}

static void PrintTripodState(const char* tag, const std::vector<MotorInfo>& tripod) {
  std::cout << tag << "\n";
  for (const auto& info : tripod) {
    float p = info.motor->getPosition();
    float v = info.motor->getVelocity();
    float t = info.motor->getTorque();
    int   e = info.motor->getErrorFlag();
    std::cout << "  id=" << info.id
              << " p=" << p
              << " v=" << v
              << " tau=" << t
              << " err=" << e
              << "\n";
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

    const float omega = 5.0f;                 // rad/s (your speed target)
    const float delta = 2.0*(float)M_PI;           // per-step angle: PI=180deg, TWO_PI=360deg
    const int reps = 1;                        // how many A/B alternations

    auto move_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<float>(delta / omega));

    const auto hold_time = std::chrono::milliseconds(1000);

    const float kd_move = 2.0f;                // damping during motion
    const float kp_hold = 1.0f;                // stiffness while holding
    const float kd_hold = 1.0f;                // damping while holding
    const auto period = std::chrono::milliseconds(10);


    for (int k = 0; k < reps; ++k) {
    std::cout << "\nStep " << k << ": Tripod A move\n";
    PrintTripodState("BEFORE A move", left_tripod);
    PrintTripodState("BEFORE A move (hold tripod)", right_tripod);
    MoveTripodByVelocity(left_tripod, right_tripod, omega, move_time,
                        kd_move, kp_hold, kd_hold);

    std::cout << "Hold both\n";
    HoldTripodAtCurrent(left_tripod, hold_time, kp_hold, kd_hold);
    HoldTripodAtCurrent(right_tripod, hold_time, kp_hold, kd_hold);

    std::cout << "Step " << k << ": Tripod B move\n";
    PrintTripodState("AFTER A move", left_tripod);
    PrintTripodState("AFTER A move (hold tripod)", right_tripod);
    MoveTripodByVelocity(right_tripod, left_tripod, omega, move_time,
                        kd_move, kp_hold, kd_hold);

    std::cout << "Hold both\n";
    HoldTripodAtCurrent(left_tripod, hold_time, kp_hold, kd_hold);
    HoldTripodAtCurrent(right_tripod, hold_time, kp_hold, kd_hold);
    }


    std::cout << "Returning ALL motors to home...\n";
    auto start_home = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_home < std::chrono::seconds(3)) {
    for (auto& info : left_tripod) {
        info.motor->sendCommandMITMode(info.home_pos, 0.0f, 2.0f, 2.0f, 0.0f);
    }
    for (auto& info : right_tripod) {
        info.motor->sendCommandMITMode(info.home_pos, 0.0f, 2.0f, 2.0f, 0.0f);
    }
    std::this_thread::sleep_for(period);
    }


    for (auto* motor : all_motors) {
        motor->exitMITMode();
    }

    return 0;
}

#ifndef CUBEMARS_PI3HAT_H
#define CUBEMARS_PI3HAT_H

#include <vector>
#include "pi3hat.h"

// CubeMars motor control via Pi3Hat CAN interface (MIT mode)
// This is separate from the SocketCAN-based CubemarsControl class
class CubemarsPi3Hat {
public:
    CubemarsPi3Hat(int motor_id, int can_bus, mjbots::pi3hat::Pi3Hat* pi3hat);

    // MIT Mode command functions
    void sendCommandMITMode(float pos, float vel, float kp, float kd, float torq);
    void enterMITMode();
    void exitMITMode();
    void zeroMotor();

    // Getters for feedback
    float getPosition() const;
    float getVelocity() const;
    float getTorque() const;
    float getTemperature() const;
    int getErrorFlag() const;

private:
    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    bool unpackReply(const mjbots::pi3hat::CanFrame &frame);

    // Parameter limits for MIT mode
    static constexpr float P_MIN = -12.5f;
    static constexpr float P_MAX = +12.5f;
    static constexpr float V_MIN = -50.0f;
    static constexpr float V_MAX = +50.0f;
    static constexpr float T_MIN = -65.0f;
    static constexpr float T_MAX = +65.0f;
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;

    int motor_id_;
    int can_bus_;
    mjbots::pi3hat::Pi3Hat* pi3hat_;

    struct MotorData {
        float position = 0.0f;
        float velocity = 0.0f;
        float torque = 0.0f;
        float temperature = 0.0f;
        int error_flag = 0;
    } motor_data_;

    std::vector<mjbots::pi3hat::CanFrame> tx_can_;
    std::vector<mjbots::pi3hat::CanFrame> rx_can_;
};

#endif // CUBEMARS_PI3HAT_H

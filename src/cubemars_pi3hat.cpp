#include "cubemars_pi3hat.h"
#include <iostream>
#include <cstring>
#include <algorithm>

CubemarsPi3Hat::CubemarsPi3Hat(int motor_id, int can_bus, mjbots::pi3hat::Pi3Hat* pi3hat)
    : motor_id_(motor_id), can_bus_(can_bus), pi3hat_(pi3hat)
{
    tx_can_.resize(5);
    rx_can_.resize(5);
}

int CubemarsPi3Hat::float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    x = std::clamp(x, x_min, x_max);
    return static_cast<int>((x - x_min) * ((1 << bits) / span));
}

float CubemarsPi3Hat::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return static_cast<float>(x_int) * span / ((1 << bits) - 1) + x_min;
}

void CubemarsPi3Hat::sendCommandMITMode(float pos, float vel, float kp, float kd, float torq) {
    float p_des  = std::clamp(pos,  P_MIN, P_MAX);
    float v_des  = std::clamp(vel,  V_MIN, V_MAX);
    float kp_des = std::clamp(kp,   KP_MIN, KP_MAX);
    float kd_des = std::clamp(kd,   KD_MIN, KD_MAX);
    float t_ff   = std::clamp(torq, T_MIN, T_MAX);

    int con_pos  = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int con_vel  = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int con_kp   = float_to_uint(kp_des, KP_MIN, KP_MAX, 12);
    int con_kd   = float_to_uint(kd_des, KD_MIN, KD_MAX, 12);
    int con_torq = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    mjbots::pi3hat::CanFrame &frame = tx_can_[can_bus_];
    frame.id = motor_id_;
    frame.bus = 5;  // JC5
    frame.size = 8;
    frame.expect_reply = true;
    frame.data[0] = static_cast<uint8_t>(con_pos >> 8);
    frame.data[1] = static_cast<uint8_t>(con_pos & 0xFF);
    frame.data[2] = static_cast<uint8_t>(con_vel >> 4);
    frame.data[3] = static_cast<uint8_t>(((con_vel & 0xF) << 4) | (con_kp >> 8));
    frame.data[4] = static_cast<uint8_t>(con_kp & 0xFF);
    frame.data[5] = static_cast<uint8_t>(con_kd >> 4);
    frame.data[6] = static_cast<uint8_t>(((con_kd & 0xF) << 4) | (con_torq >> 8));
    frame.data[7] = static_cast<uint8_t>(con_torq & 0xFF);

    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
    input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
    input.force_can_check = (1 << 5);
    input.timeout_ns = 1200000;

    auto output = pi3hat_->Cycle(input);
    for (size_t i = 0; i < output.rx_can_size; i++) {
        if ((rx_can_[i].id & 0x7FF) == static_cast<uint32_t>(motor_id_) ||
            rx_can_[i].data[0] == static_cast<uint8_t>(motor_id_)) {
            unpackReply(rx_can_[i]);
        }
    }
}

void CubemarsPi3Hat::enterMITMode() {
    mjbots::pi3hat::CanFrame &frame = tx_can_[can_bus_];
    frame.id = motor_id_;
    frame.bus = 5;
    frame.size = 8;
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFC;
    frame.expect_reply = false;

    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
    input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
    input.force_can_check = (1 << 5);
    input.timeout_ns = 1200000;

    pi3hat_->Cycle(input);
}

void CubemarsPi3Hat::exitMITMode() {
    mjbots::pi3hat::CanFrame &frame = tx_can_[can_bus_];
    frame.id = motor_id_;
    frame.bus = 5;
    frame.size = 8;
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFD;
    frame.expect_reply = false;

    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
    input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
    input.force_can_check = (1 << 5);
    input.timeout_ns = 1200000;

    pi3hat_->Cycle(input);
}

void CubemarsPi3Hat::zeroMotor() {
    mjbots::pi3hat::CanFrame &frame = tx_can_[can_bus_];
    frame.id = motor_id_;
    frame.bus = 5;
    frame.size = 8;
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFE;
    frame.expect_reply = false;

    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
    input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
    input.force_can_check = (1 << 5);
    input.timeout_ns = 1200000;

    pi3hat_->Cycle(input);
}

bool CubemarsPi3Hat::unpackReply(const mjbots::pi3hat::CanFrame &frame) {
    if (frame.size != 8)
        return false;

    if (frame.data[0] != static_cast<uint8_t>(motor_id_)) {
        return false;
    }

    int p_int = (frame.data[1] << 8) | frame.data[2];
    int v_int = (frame.data[3] << 4) | (frame.data[4] >> 4);
    int i_int = ((frame.data[4] & 0x0F) << 8) | frame.data[5];

    motor_data_.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    motor_data_.velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
    motor_data_.torque   = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    motor_data_.temperature = static_cast<float>(frame.data[6]) - 40;
    motor_data_.error_flag = frame.data[7];

    return true;
}

float CubemarsPi3Hat::getPosition() const { return motor_data_.position; }
float CubemarsPi3Hat::getVelocity() const { return motor_data_.velocity; }
float CubemarsPi3Hat::getTorque() const { return motor_data_.torque; }
float CubemarsPi3Hat::getTemperature() const { return motor_data_.temperature; }
int CubemarsPi3Hat::getErrorFlag() const { return motor_data_.error_flag; }

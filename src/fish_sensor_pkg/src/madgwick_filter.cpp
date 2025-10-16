#include "fish_sensor_pkg/madgwick_filter.hpp"
#include <cmath>

MadgwickFilter::MadgwickFilter() = default;

void MadgwickFilter::update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
    // ローカル変数にコピー
    double q0 = q0_, q1 = q1_, q2 = q2_, q3 = q3_;
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 加速度がゼロベクトルならジャイロのみで更新
    if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
        // 正規化
        recipNorm = 1.0 / std::sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 補助変数
        _2q0 = 2.0 * q0;
        _2q1 = 2.0 * q1;
        _2q2 = 2.0 * q2;
        _2q3 = 2.0 * q3;
        _4q0 = 4.0 * q0;
        _4q1 = 4.0 * q1;
        _4q2 = 4.0 * q2;
        _8q1 = 8.0 * q1;
        _8q2 = 8.0 * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // 勾配降下法
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
        recipNorm = 1.0 / std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 姿勢誤差をジャイロの測定値に適用
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
        
        qDot1 -= beta_ * s0;
        qDot2 -= beta_ * s1;
        qDot3 -= beta_ * s2;
        qDot4 -= beta_ * s3;
    } else {
        // ジャイロのみで更新
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
    }

    // 四元数を積分
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 四元数を正規化
    recipNorm = 1.0 / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0_ = q0 * recipNorm;
    q1_ = q1 * recipNorm;
    q2_ = q2 * recipNorm;
    q3_ = q3 * recipNorm;
}

std::array<double, 4> MadgwickFilter::getQuaternion() const {
    return {q0_, q1_, q2_, q3_}; // w, x, y, z
}

geometry_msgs::msg::Vector3 MadgwickFilter::getLinearAcceleration(double ax, double ay, double az) const {
    // 重力ベクトル [m/s^2]
    const double G = 9.80665;
    double gx = G * (2 * (q1_ * q3_ - q0_ * q2_));
    double gy = G * (2 * (q0_ * q1_ + q2_ * q3_));
    double gz = G * (q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_);

    geometry_msgs::msg::Vector3 linear_accel;
    linear_accel.x = ax - gx;
    linear_accel.y = ay - gy;
    linear_accel.z = az - gz;

    return linear_accel;
}
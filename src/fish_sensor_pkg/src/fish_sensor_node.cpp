#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "fish_sensor_pkg/mpu6500.hpp"
#include "fish_sensor_pkg/as5600.hpp"
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

struct FishingRod {
    int id;
    std::string frame_id;
    std::unique_ptr<MPU6500> mpu;
    std::unique_ptr<AS5600> as;
    bool mpu_active = false;
    bool as_active = false;

    // ドリフト補正用
    double gyro_offset_x = 0.0;
    double gyro_offset_y = 0.0;
    double gyro_offset_z = 0.0;

    double last_angle_deg = 0.0;
    bool first_angle_reading = true;
};

class FishSensorNode : public rclcpp::Node {
public:
    FishSensorNode() : Node("fish_sensor_node") {
        this->declare_parameter<std::vector<std::string>>("rod_ids", std::vector<std::string>{});
        auto rod_ids = this->get_parameter("rod_ids").as_string_array();

        for (const auto& id_str : rod_ids) {
            int id = std::stoi(id_str);
            this->declare_parameter<std::string>("rod_" + id_str + ".i2c_bus_mpu", "/dev/i2c-3");
            this->declare_parameter<int>("rod_" + id_str + ".i2c_addr_mpu", 0x68);
            this->declare_parameter<std::string>("rod_" + id_str + ".i2c_bus_as", "/dev/i2c-3");
            this->declare_parameter<int>("rod_" + id_str + ".i2c_addr_as", 0x36);

            auto rod = std::make_unique<FishingRod>();
            rod->id = id;
            rod->frame_id = id_str;
            rod->mpu = std::make_unique<MPU6500>(
                this->get_parameter("rod_" + id_str + ".i2c_bus_mpu").as_string(),
                this->get_parameter("rod_" + id_str + ".i2c_addr_mpu").as_int()
            );
            rod->as = std::make_unique<AS5600>(
                this->get_parameter("rod_" + id_str + ".i2c_bus_as").as_string(),
                this->get_parameter("rod_" + id_str + ".i2c_addr_as").as_int()
            );
            rods_.push_back(std::move(rod));
        }

        initialize_sensors();

        // Publisher
        raw_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/fish/imu/data_raw", 10);
        rotation_pub_ = this->create_publisher<std_msgs::msg::String>("/fish/ctrl/out", 10);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/fish/imu/yaw_angle", 10);

        timer_ = this->create_wall_timer(10ms, std::bind(&FishSensorNode::timer_callback, this));
        last_time_ = this->now();
    }

private:
    // ------------------------------------------
    // センサー初期化 & キャリブレーション
    // ------------------------------------------
    void initialize_sensors() {
        for (auto& rod : rods_) {
            RCLCPP_INFO(this->get_logger(), "Initializing Rod ID: %d", rod->id);

            // MPU
            if (rod->mpu->init()) {
                rod->mpu_active = true;
                RCLCPP_INFO(this->get_logger(), "  -> MPU6500 connected. Calibrating...");

                // ジャイロオフセット測定（静止状態で約1秒）
                double gx_sum = 0, gy_sum = 0, gz_sum = 0;
                const int sample_count = 100;
                for (int i = 0; i < sample_count; ++i) {
                    auto d = rod->mpu->readSensorData();
                    gx_sum += d.gx;
                    gy_sum += d.gy;
                    gz_sum += d.gz;
                    rclcpp::sleep_for(10ms);
                }
                rod->gyro_offset_x = gx_sum / sample_count;
                rod->gyro_offset_y = gy_sum / sample_count;
                rod->gyro_offset_z = gz_sum / sample_count;

                RCLCPP_INFO(this->get_logger(),
                            "  -> Gyro offset: (%.5f, %.5f, %.5f)",
                            rod->gyro_offset_x, rod->gyro_offset_y, rod->gyro_offset_z);
            } else {
                rod->mpu_active = false;
                RCLCPP_WARN(this->get_logger(), "  -> MPU6500 not found.");
            }

            // AS5600
            if (rod->as->init()) {
                rod->as_active = true;
                RCLCPP_INFO(this->get_logger(), "  -> AS5600 connected.");
            } else {
                rod->as_active = false;
                RCLCPP_WARN(this->get_logger(), "  -> AS5600 not found.");
            }
        }
    }

    // ------------------------------------------
    // メインループ
    // ------------------------------------------
    void timer_callback() {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0) return;

        for (auto& rod : rods_) {
            if (rod->mpu_active) {
                try {
                    ImuData raw = rod->mpu->readSensorData();

                    // オフセット補正
                    raw.gx -= rod->gyro_offset_x;
                    raw.gy -= rod->gyro_offset_y;
                    raw.gz -= rod->gyro_offset_z;

                    sensor_msgs::msg::Imu msg;
                    msg.header.stamp = current_time;
                    msg.header.frame_id = rod->frame_id;

                    msg.orientation.w = 1.0;
                    msg.orientation.x = 0.0;
                    msg.orientation.y = 0.0;
                    msg.orientation.z = 0.0;

                    msg.angular_velocity.x = raw.gx;
                    msg.angular_velocity.y = raw.gy;
                    msg.angular_velocity.z = raw.gz;

                    msg.linear_acceleration.x = raw.ax;
                    msg.linear_acceleration.y = raw.ay;
                    msg.linear_acceleration.z = raw.az;

                    raw_imu_pub_->publish(msg);

                    // ヨー角積分
                    yaw_angle_ += raw.gz * dt;
                    std_msgs::msg::Float64 yaw_msg;
                    yaw_msg.data = yaw_angle_;
                    yaw_pub_->publish(yaw_msg);

                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Error reading MPU6500 (rod %d): %s",
                                 rod->id, e.what());
                }
            }

            // AS5600 処理（回転速度）
            if (rod->as_active) {
                try {
                    double ang = rod->as->getAngleDeg();
                    if (rod->first_angle_reading) {
                        rod->last_angle_deg = ang;
                        rod->first_angle_reading = false;
                    } else {
                        double diff = ang - rod->last_angle_deg;
                        if (diff > 180.0) diff -= 360.0;
                        if (diff < -180.0) diff += 360.0;

                        // ★ 角度差をラジアンに変換してから速度計算
                        double diff_rad = diff * M_PI / 180.0;
                        double rot_speed = diff_rad / dt;  // rad/s

                        std_msgs::msg::String msg;
                        msg.data = rod->frame_id + "," + std::to_string(rot_speed);
                        rotation_pub_->publish(msg);

                        rod->last_angle_deg = ang;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(),
                                "Error reading AS5600 (rod %d): %s",
                                rod->id, e.what());
                }
            }

        }

        last_time_ = current_time;
    }

    // ------------------------------------------
    // メンバ変数
    // ------------------------------------------
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rotation_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
    std::vector<std::unique_ptr<FishingRod>> rods_;
    rclcpp::Time last_time_;
    double yaw_angle_ = 0.0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FishSensorNode>());
    rclcpp::shutdown();
    return 0;
}

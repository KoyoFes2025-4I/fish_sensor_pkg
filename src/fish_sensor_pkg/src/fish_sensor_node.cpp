#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "fish_sensor_pkg/mpu6500.hpp"
#include "fish_sensor_pkg/as5600.hpp"
#include "fish_sensor_pkg/madgwick_filter.hpp"
#include <memory>
#include <vector>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

struct FishingRod {
    int id;
    std::string frame_id;
    std::unique_ptr<MPU6500> mpu;
    std::unique_ptr<AS5600> as;
    std::unique_ptr<MadgwickFilter> filter;
    
    bool mpu_active = false;
    bool as_active = false;

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

            std::string mpu_bus = this->get_parameter("rod_" + id_str + ".i2c_bus_mpu").as_string();
            int mpu_addr = this->get_parameter("rod_" + id_str + ".i2c_addr_mpu").as_int();
            rod->mpu = std::make_unique<MPU6500>(mpu_bus, mpu_addr, this->get_logger());

            std::string as_bus = this->get_parameter("rod_" + id_str + ".i2c_bus_as").as_string();
            int as_addr = this->get_parameter("rod_" + id_str + ".i2c_addr_as").as_int();
            rod->as = std::make_unique<AS5600>(as_bus, as_addr);
            
            rod->filter = std::make_unique<MadgwickFilter>();
            
            rods_.push_back(std::move(rod));
        }

        // センサーの初期化
        initialize_sensors();

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/fish/ctrl/imu", 10);
        rotation_pub_ = this->create_publisher<std_msgs::msg::String>("/fish/ctrl/out", 10);

        // 100Hzでタイマーを設定
        timer_ = this->create_wall_timer(10ms, std::bind(&FishSensorNode::timer_callback, this));
        last_time_ = this->now();
    }

private:
    void initialize_sensors() {
        for (auto& rod : rods_) {
            RCLCPP_INFO(this->get_logger(), "Initializing Rod ID: %d", rod->id);
            
            // MPU6500初期化
            if (rod->mpu->init()) {
                rod->mpu_active = true;
                RCLCPP_INFO(this->get_logger(), "  -> MPU6500 connection successful.");
            } else {
                rod->mpu_active = false;
                RCLCPP_WARN(this->get_logger(), "  -> MPU6500 not found or failed to initialize.");
            }

            // AS5600初期化
            if (rod->as->init()) {
                rod->as_active = true;
                RCLCPP_INFO(this->get_logger(), "  -> AS5600 connection successful.");
            } else {
                rod->as_active = false;
                RCLCPP_WARN(this->get_logger(), "  -> AS5600 not found or failed to initialize.");
            }
        }
    }

    void timer_callback() {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0) return;

        for (auto& rod : rods_) {
            // MPU6500処理
            if (rod->mpu_active) {
                try {
                    ImuData raw_data = rod->mpu->readSensorData();
                    rod->filter->update(raw_data.gx, raw_data.gy, raw_data.gz,
                                        raw_data.ax, raw_data.ay, raw_data.az, dt);
                    
                    auto quat = rod->filter->getQuaternion();
                    auto linear_accel = rod->filter->getLinearAcceleration(raw_data.ax, raw_data.ay, raw_data.az);

                    sensor_msgs::msg::Imu imu_msg;
                    imu_msg.header.stamp = current_time;
                    imu_msg.header.frame_id = rod->frame_id;
                    
                    imu_msg.orientation.w = quat[0];
                    imu_msg.orientation.x = quat[1];
                    imu_msg.orientation.y = quat[2];
                    imu_msg.orientation.z = quat[3];

                    imu_msg.linear_acceleration.x = linear_accel.x;
                    imu_msg.linear_acceleration.y = linear_accel.y;
                    imu_msg.linear_acceleration.z = linear_accel.z;
                    
                    // 共分散は未設定（0）
                    
                    imu_pub_->publish(imu_msg);

                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error reading MPU6500 for rod %d: %s", rod->id, e.what());
                }
            }

            // AS5600処理
            if (rod->as_active) {
                try {
                    double current_angle = rod->as->getAngleDeg();

                    if (rod->first_angle_reading) {
                        rod->last_angle_deg = current_angle;
                        rod->first_angle_reading = false;
                    } else {
                        double angle_diff = current_angle - rod->last_angle_deg;

                        // 角度のラップアラウンド処理 (-180, 180)
                        if (angle_diff > 180.0) {
                            angle_diff -= 360.0;
                        } else if (angle_diff < -180.0) {
                            angle_diff += 360.0;
                        }

                        double rotation_speed = angle_diff / dt; // deg/s

                        std_msgs::msg::String rotation_msg;
                        rotation_msg.data = rod->frame_id + "," + std::to_string(rotation_speed);
                        rotation_pub_->publish(rotation_msg);

                        rod->last_angle_deg = current_angle;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error reading AS5600 for rod %d: %s", rod->id, e.what());
                }
            }
        }

        last_time_ = current_time;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rotation_pub_;
    
    std::vector<std::unique_ptr<FishingRod>> rods_;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FishSensorNode>());
    rclcpp::shutdown();
    return 0;
}
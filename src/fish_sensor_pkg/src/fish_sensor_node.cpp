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

struct FishingRod
{
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

    // AS5600用
    double last_angle_deg = 0.0;
    bool first_angle_reading = true;

    // ヨー角積分用
    double yaw_angle = 0.0;

    // ロッド専用のPublisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotation_publisher;

    // ★ 修正点 1: IMU生データPublisherをロッドごとに追加
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_publisher;
};

class FishSensorNode : public rclcpp::Node
{
public:
    FishSensorNode() : Node("fish_sensor_node")
    {
        this->declare_parameter<std::vector<std::string>>("rod_ids", std::vector<std::string>{});
        auto rod_ids = this->get_parameter("rod_ids").as_string_array();

        for (const auto &id_str : rod_ids)
        {
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
                this->get_parameter("rod_" + id_str + ".i2c_addr_mpu").as_int());
            rod->as = std::make_unique<AS5600>(
                this->get_parameter("rod_" + id_str + ".i2c_bus_as").as_string(),
                this->get_parameter("rod_" + id_str + ".i2c_addr_as").as_int());

            // ヨー角Publisher
            std::string yaw_topic_name = "/fish/imu/yaw_angle/rod_" + rod->frame_id;
            rod->yaw_publisher = this->create_publisher<std_msgs::msg::Float64>(yaw_topic_name, 10);

            // 回転速度Publisher (Float64)
            std::string rot_topic_name = "/fish/ctrl/rotation_speed/rod_" + rod->frame_id;
            rod->rotation_publisher = this->create_publisher<std_msgs::msg::Float64>(rot_topic_name, 10);

            // ★ 修正点 2: IMU生データPublisherをIDごとに作成
            std::string imu_topic_name = "/fish/imu/data_raw/rod_" + rod->frame_id;
            rod->raw_imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, 10);

            rods_.push_back(std::move(rod));
        }

        initialize_sensors();

        // Publisher
        // ★ 修正点 3: 共有のIMU Publisherを削除
        // raw_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/fish/imu/data_raw", 10);
        rotation_pub_ = this->create_publisher<std_msgs::msg::String>("/fish/ctrl/out", 10);

        timer_ = this->create_wall_timer(30ms, std::bind(&FishSensorNode::timer_callback, this));
        last_time_ = this->now();
    }

private:
    // ------------------------------------------
    // センサー初期化 & キャリブレーション (変更なし)
    // ------------------------------------------
    void initialize_sensors()
    {
        for (auto &rod : rods_)
        {
            RCLCPP_INFO(this->get_logger(), "Initializing Rod ID: %d", rod->id);
            // ... (MPUとAS5600の初期化コードは変更なし) ...
            // MPU
            if (rod->mpu->init())
            {
                rod->mpu_active = true;
                RCLCPP_INFO(this->get_logger(), "  -> MPU6500 connected. Calibrating...");

                // ジャイロオフセット測定
                double gx_sum = 0, gy_sum = 0, gz_sum = 0;
                const int sample_count = 100;
                for (int i = 0; i < sample_count; ++i)
                {
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
            }
            else
            {
                rod->mpu_active = false;
                RCLCPP_WARN(this->get_logger(), "  -> MPU6500 not found.");
            }

            // AS5600
            if (rod->as->init())
            {
                rod->as_active = true;
                RCLCPP_INFO(this->get_logger(), "  -> AS5600 connected.");
            }
            else
            {
                rod->as_active = false;
                RCLCPP_WARN(this->get_logger(), "  -> AS5600 not found.");
            }
        }
    }

    // ------------------------------------------
    // メインループ
    // ------------------------------------------
    void timer_callback()
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0)
            return;

        // 全ロッドのIMU処理
        for (auto &rod : rods_)
        {
            if (rod->mpu_active)
            {
                try
                {
                    ImuData raw = rod->mpu->readSensorData();

                    // オフセット補正
                    raw.gx -= rod->gyro_offset_x;
                    raw.gy -= rod->gyro_offset_y;
                    raw.gz -= rod->gyro_offset_z;

                    sensor_msgs::msg::Imu msg;
                    msg.header.stamp = current_time;
                    msg.header.frame_id = rod->frame_id;

                    msg.orientation.w = 1.0;
                    msg.angular_velocity.x = raw.gx;
                    msg.angular_velocity.y = raw.gy;
                    msg.angular_velocity.z = raw.gz;
                    msg.linear_acceleration.x = raw.ax;
                    msg.linear_acceleration.y = raw.ay;
                    msg.linear_acceleration.z = raw.az;

                    // ★ 修正点 4: ロッド専用のPublisherから発行
                    rod->raw_imu_publisher->publish(msg);

                    // ヨー角積分
                    rod->yaw_angle += raw.gz * dt;
                    std_msgs::msg::Float64 yaw_msg;
                    yaw_msg.data = rod->yaw_angle;

                    // rod->yaw_publisher->publish(yaw_msg);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Error reading MPU6500 (rod %d): %s",
                                 rod->id, e.what());
                }
            }
        }

        // 全ロッドのAS5600処理 (変更なし)
        for (auto &rod : rods_)
        {
            if (rod->as_active)
            {
                try
                {
                    double ang = rod->as->getAngleDeg();
                    if (rod->first_angle_reading)
                    {
                        rod->last_angle_deg = ang;
                        rod->first_angle_reading = false;
                    }
                    else
                    {
                        double diff = ang - rod->last_angle_deg;
                        if (diff > 180.0)
                            diff -= 360.0;
                        if (diff < -180.0)
                            diff += 360.0;

                        double diff_rad = diff * M_PI / 180.0;
                        double rot_speed = diff_rad / dt; // rad/s

                        // 要件1: "id,rotation" の String型でPublish
                        std_msgs::msg::String string_msg;
                        string_msg.data = rod->frame_id + "," + std::to_string(rot_speed);
                        rotation_pub_->publish(string_msg);

                        // 要件2: Float64型で専用トピックにPublish
                        std_msgs::msg::Float64 float_msg;
                        float_msg.data = rot_speed;
                        // rod->rotation_publisher->publish(float_msg);

                        rod->last_angle_deg = ang;
                    }
                }
                catch (const std::exception &e)
                {
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
    // ★ 修正点 3: 共有のIMU Publisherを削除
    // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rotation_pub_; // String型
    std::vector<std::unique_ptr<FishingRod>> rods_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FishSensorNode>());
    rclcpp::shutdown();
    return 0;
}
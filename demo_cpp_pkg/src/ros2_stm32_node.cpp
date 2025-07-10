#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <serial/serial.h>
#include <chrono>
#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

/**
 * @class Stm32RobotController
 * @brief STM32机器人控制节点，负责ROS与STM32的串口通信
 * 
 * 功能包括：
 * - 接收cmd_vel速度指令并转发给STM32
 * - 解析STM32上传的传感器数据
 * - 发布里程计、IMU和电源数据
 */
class Stm32RobotController : public rclcpp::Node {
public:
    Stm32RobotController() : Node("stm32_robot_controller") {
        /* 参数声明（可在launch文件中配置）*/
        declare_parameter("serial_port", "/dev/ttyUSB0");    // 串口设备路径
        declare_parameter("baud_rate", 115200);             // 串口波特率
        declare_parameter("max_linear_vel", 1.0);           // 最大线速度(m/s)
        declare_parameter("max_angular_vel", 1.0);          // 最大角速度(rad/s)
        declare_parameter("wheel_radius", 0.065);           // 车轮半径(m)
        declare_parameter("wheel_base", 0.485);             // 轮距(m)
        declare_parameter("default_flag1", 0x00);           // 控制标志位1
        declare_parameter("default_flag2", 0x00);           // 控制标志位2

        /* 参数获取 */
        serial_port_ = get_parameter("serial_port").as_string();
        baud_rate_ = get_parameter("baud_rate").as_int();
        max_linear_vel_ = get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = get_parameter("max_angular_vel").as_double();
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        wheel_base_ = get_parameter("wheel_base").as_double();
        default_flag1_ = get_parameter("default_flag1").as_int();
        default_flag2_ = get_parameter("default_flag2").as_int();

        /* 状态初始化 */
        x_ = y_ = yaw_ = 0.0;                  // 初始位姿
        linear_vel_x = angular_vel_z = 0.0;       // 初始速度
        power_level_ = 0.0;                     // 初始电量
        imu_yaw_ = imu_wz_ = imu_az_ = 0.0;     // 初始IMU数据
        last_odom_time_ = this->now();          // 初始化时间戳
        last_tx_time_ = std::chrono::steady_clock::now();

        // 初始化串口
        init_serial();

        /* ROS接口初始化 */
        // 发布器
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
        power_pub_ = create_publisher<std_msgs::msg::Float32>("power_level", 10);
        
        // 订阅器（绑定cmd_vel回调函数）
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Stm32RobotController::cmd_vel_callback, 
                                   this, std::placeholders::_1));

        // 定时器（10ms周期读取串口数据）
        read_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Stm32RobotController::read_serial_data, this));
            
        RCLCPP_INFO(get_logger(), "STM32机器人控制器已启动");
    }

    ~Stm32RobotController() {
        if (serial_.isOpen()) {
            serial_.close();  // 节点关闭时释放串口
        }
    }

private:
    /* 硬件通信相关成员变量 */
    serial::Serial serial_;          // 串口对象
    std::string serial_port_;        // 串口设备路径
    int baud_rate_;                  // 波特率
    std::vector<uint8_t> buffer_;    // 串口数据缓冲区

    /* 运动控制参数 */
    double linear_vel_x;             // 当前线速度(m/s)        
    double angular_vel_z;             // 当前角速度(rad/s)
    double max_linear_vel_;          // 最大线速度
    double max_angular_vel_;         // 最大角速度
    uint8_t default_flag1_;          // 控制标志位1
    uint8_t default_flag2_;          // 控制标志位2

    /* 机器人状态 */
    double x_, y_, yaw_;             // 里程计位姿
    double wheel_radius_;            // 车轮半径
    double wheel_base_;              // 轮距
    double power_level_;             // 电源电压
    double imu_yaw_, imu_wz_, imu_az_; // IMU数据

    /* 时间记录 */
    rclcpp::Time last_odom_time_;    // 上次里程计更新时间
    std::chrono::time_point<std::chrono::steady_clock> last_tx_time_; // 上次发送时间

    /* ROS接口 */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr power_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr read_timer_;

    /**
     * @brief 初始化串口
     * @note 失败时会直接关闭ROS节点
     */
    void init_serial() {
        try {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(timeout);
            serial_.open();
            RCLCPP_INFO(get_logger(), "成功打开串口: %s, 波特率: %d", 
                       serial_port_.c_str(), baud_rate_);
        } catch (const serial::SerialException& e) {
            RCLCPP_FATAL(get_logger(), "串口打开失败: %s", e.what());
            rclcpp::shutdown();  // 串口初始化失败直接关闭节点
        }
    }

    /**
     * @brief 计算异或校验和
     * @param buffer 数据指针
     * @param length 数据长度
     * @return 校验和结果
     */
  uint8_t calculate_checksum(const uint8_t* buffer, size_t length) {
    uint8_t check_sum = 0;
    for(size_t k = 0; k < length; k++) {
        check_sum ^= buffer[k];
    }
    return check_sum;
}

    /**
     * @brief cmd_vel话题回调函数
     * @param msg 速度指令消息
     * @note 限制速度范围并通过定时器控制发送频率
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 限制速度范围
        linear_vel_x = std::clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
        angular_vel_z = std::clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

        // linear_vel_z = std::clamp(msg->linear.z, -max_linear_vel_, max_linear_vel_);
        
        // // 发送频率控制（50ms间隔）
        // auto now = std::chrono::steady_clock::now();
        // if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tx_time_).count() < 50) {
        //     return;
        // }
        // last_tx_time_ = now;
        
        send_velocity_command();  // 发送速度指令
    }

    /**
     * @brief 发送速度指令到STM32
     * @note 协议格式：[0x7B][Flag1][Flag2][VX高][VX低][VY高][VY低][VZ高][VZ低][0x00][校验和][0x7D]
     */
    void send_velocity_command() {
        uint8_t buffer[11] = {0};  // 初始化缓冲区
        // 帧头 帧尾
        buffer[0]=0x7B;
        buffer[10]=0x7D;
        // 设置控制标志位
        buffer[1] = default_flag1_;
        buffer[2] = default_flag2_;
        
        // 速度单位转换（m/s→mm/s, rad/s→mrad/s）
        int16_t vx = static_cast<int16_t>(linear_vel_x* 1000);
        int16_t vz = static_cast<int16_t>(angular_vel_z* 1000);
        int16_t vy = 0;  // 差分驱动机器人vy固定为0
        // int16_t vz = static_cast<int16_t>(angular_vel_ * 1000);
        printf("vz=%dr\n",vz);
        // 填充速度数据（大端序）
        buffer[3] = (vx >> 8) & 0xFF;  // VX高字节
        buffer[4] = vx & 0xFF;         // VX低字节
        buffer[5] = (vy >> 8) & 0xFF;  // VY高字节
        buffer[6] = vy & 0xFF;         // VY低字节
        buffer[7] = (vz >> 8) & 0xFF;  // VZ高字节
        buffer[8] = vz & 0xFF;         // VZ低字节
        // 计算校验和（从Flag1到保留字节共8字节）
        buffer[9] = calculate_checksum(buffer, 9);
        
        // 发送数据
        try {
            if (!serial_.isOpen()) {
                init_serial();  // 尝试重新初始化
                if (!serial_.isOpen()) return;
            }
            serial_.write(buffer, sizeof(buffer));
        } catch (const serial::SerialException& e) {
            RCLCPP_ERROR(get_logger(), "串口发送失败: %s", e.what());
        }
    }

    /**
     * @brief 读取并解析串口数据
     * @note 使用循环缓冲区处理数据，支持断帧续传
     */
    void read_serial_data() {
    try {
        if (!serial_.isOpen() || !serial_.available()) return;
        
        std::string data = serial_.read(serial_.available());
        buffer_.insert(buffer_.end(), data.begin(), data.end());
        
        // 处理18字节完整数据帧
        while (buffer_.size() >= 18) {
            // 查找帧头0x5B
            auto it = std::find(buffer_.begin(), buffer_.end(), 0x5B);
            if (it == buffer_.end()) {
                buffer_.clear();
                break;
            }
            
            size_t remaining = std::distance(it, buffer_.end());
            
            // 检查18字节帧尾和长度
            if (remaining >= 18 && *(it + 17) == 0x5D) {
                if (verify_checksum(it)) {
                    parse_robot_data(it);
                    buffer_.erase(buffer_.begin(), it + 18);
                    continue;
                }
            }
            
            // 移除无效帧头
            buffer_.erase(buffer_.begin(), it + 1);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "串口读取错误: %s", e.what());
        buffer_.clear();
    }
}

bool verify_checksum(std::vector<uint8_t>::iterator it) {
    uint8_t received_checksum = *(it + 16);  // 校验和在第17字节(索引16)
    uint8_t calculated_checksum = calculate_checksum(&(*it), 16);  // 计算前16字节
    return received_checksum == calculated_checksum;
}


void parse_robot_data(std::vector<uint8_t>::iterator it) {
    // 解析速度和IMU数据
    int16_t vx = (*(it + 2) << 8) | *(it + 3);
    int16_t vz = (*(it + 6) << 8) | *(it + 7);
    int16_t power = (*(it + 8) << 8) | *(it + 9);
    int16_t yaw = (*(it + 10) << 8) | *(it + 11);
    int16_t wz = (*(it + 12) << 8) | *(it + 13);
    int16_t az = (*(it + 14) << 8) | *(it + 15);
    
    // 单位转换
    double linear_vel = vx / 1000.0;
    double angular_vel = vz / 1000.0;
    power_level_ = power / 1000.0;
    imu_yaw_ = yaw * M_PI / 1800.0;
    imu_wz_ = wz / 1000.0;
    imu_az_ = az / 1000.0;
    
    // 发布数据
    process_odometry(linear_vel, angular_vel);
    publish_imu_data();
    
    auto power_msg = std_msgs::msg::Float32();
    power_msg.data = power_level_;
    power_pub_->publish(power_msg);
}

    /**
     * @brief 处理里程计数据
     * @param linear_vel 线速度(m/s)
     * @param angular_vel 角速度(rad/s)
     */
    void process_odometry(double linear_vel, double angular_vel) {
        auto now = this->now();
        double dt = (now - last_odom_time_).seconds();
        if (dt < 0.001) return;  // 时间间隔过小跳过
        
        // 更新位姿（速度积分）
        yaw_ += angular_vel * dt;
        x_ += linear_vel * cos(yaw_) * dt;
        y_ += linear_vel * sin(yaw_) * dt;
        last_odom_time_ = now;
        
        publish_odometry(linear_vel, angular_vel, now);
    }

    /**
     * @brief 发布里程计信息
     */
    void publish_odometry(double linear_vel, double angular_vel, rclcpp::Time stamp) {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";
        
        // 位置信息
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        
        // 姿态信息（欧拉角转四元数）
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // 速度信息
        odom_msg.twist.twist.linear.x = linear_vel;
        odom_msg.twist.twist.angular.z = angular_vel;
        
        odom_pub_->publish(odom_msg);
    }

    /**
     * @brief 发布IMU数据
     */
    void publish_imu_data() {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";
        
        // 姿态信息
        tf2::Quaternion q;
        q.setRPY(0, 0, imu_yaw_);
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
        
        // 角速度和加速度（仅Z轴）
        imu_msg.angular_velocity.z = imu_wz_;
        imu_msg.linear_acceleration.z = imu_az_;
        
        // 设置协方差表示未知
        imu_msg.orientation_covariance[0] = -1;
        imu_msg.angular_velocity_covariance[0] = -1;
        imu_msg.linear_acceleration_covariance[0] = -1;
        
        imu_pub_->publish(imu_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Stm32RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
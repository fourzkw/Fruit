#include "serial_sender/serial_sender.hpp"
#include "serial_sender/struct_serializer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <memory>
#include <thread>
#include <mutex>

// 发送给下位机的结构体
struct MessageData
{
    //舵机角度以其连接的臂竖直向上为0度
    //车辆坐标系以前后为X轴，左右为Y轴，垂直地面为Z轴
    
    float servo1;    //舵机1角度
    float servo2;    //舵机2角度
    float servo3;    //舵机3角度
    float servo4;    //舵机4角度
    bool is_grabing; //是否正在抓取，1为爪子闭合，0为张开
    float target_distance;    //当前目标前进的距离
    float target_angle;       //车辆yaw轴目标角度
};

// 从下位机接收的结构体
struct FeedbackData
{
    //舵机角度以其连接的臂竖直向上为0度，由下至上每个关节的控制舵机序号递增
    //车辆坐标系以前后为X轴，左右为Y轴，垂直地面为Z轴
    //编码器编号
    //1 2
    //3 4
    
    float servo1;    //舵机1角度
    float servo2;    //舵机2角度
    float servo3;    //舵机3角度
    float servo4;    //舵机4角度
    float sensor1;   //编码器1
    float sensor2;   //编码器2
    float sensor3;   //编码器3
    float sensor4;   //编码器4
    float imu_yaw;   //imu的yaw轴数据
};

// 数据接收线程函数
void receiveThread(const std::shared_ptr<portable_serial_sender::SerialSender>& serial_sender, 
                  FeedbackData& feedback, 
                  std::mutex& feedback_mutex,
                  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_init_pub)
{
    std::vector<uint8_t> buffer(1024);
    size_t bytes_read = 0;
    
    while(rclcpp::ok())
    {
        try 
        {
            // 从串口读取数据
            bytes_read = serial_sender->readSerial(buffer);
            
            if(bytes_read > 0)
            {
                // 检查起始标记
                if(buffer[0] == 0xAA)
                {
                    size_t expected_size = sizeof(FeedbackData);
                    
                    // 检查数据长度是否符合预期，需要起始标记、数据和结束标记
                    if(bytes_read >= expected_size + 2 && buffer[expected_size + 1] == 0xDD) // 2 = 1(起始标记) + 1(结束标记)
                    {
                        // 使用互斥锁保护共享数据
                        std::lock_guard<std::mutex> lock(feedback_mutex);
                        
                        // 将数据复制到结构体，跳过起始标记
                        std::memcpy(&feedback, &buffer[1], expected_size);
                        
                        RCLCPP_INFO(serial_sender->get_logger(), 
                            "接收到数据: servo1=%.2f, servo2=%.2f, servo3=%.2f, servo4=%.2f, "
                            "sensor1=%.2f, sensor2=%.2f, sensor3=%.2f, sensor4=%.2f, imu_yaw=%.2f",
                            feedback.servo1, feedback.servo2, feedback.servo3, feedback.servo4,
                            feedback.sensor1, feedback.sensor2, feedback.sensor3, feedback.sensor4,
                            feedback.imu_yaw);
                        
                        // 创建消息并发布到/data_init话题
                        std_msgs::msg::Float32MultiArray data_msg;
                        data_msg.data = {
                            feedback.servo1, feedback.servo2, feedback.servo3, feedback.servo4,
                            feedback.sensor1, feedback.sensor2, feedback.sensor3, feedback.sensor4,
                            feedback.imu_yaw
                        };
                        
                        // 发布消息
                        data_init_pub->publish(data_msg);
                        RCLCPP_DEBUG(serial_sender->get_logger(), "已发布数据到/data_init话题");
                    }
                    else
                    {
                        RCLCPP_WARN(serial_sender->get_logger(), "接收到的数据大小不符合预期: %zu, 需要 %zu", 
                            bytes_read, expected_size + 2);
                    }
                }
                else
                {
                    RCLCPP_WARN(serial_sender->get_logger(), "数据格式错误：未找到正确的起始标记");
                }
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(serial_sender->get_logger(), "读取串口数据错误: %s", e.what());
        }
        
        // 短暂休眠，避免占用过多CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// 舵机角度回调函数
void servoAngleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg, MessageData& message_data)
{
    // 检查消息数据长度是否足够
    if (msg->data.size() >= 4) {
        message_data.servo1 = msg->data[0];
        message_data.servo2 = msg->data[1];
        message_data.servo3 = msg->data[2];
        message_data.servo4 = msg->data[3];
    }
}

int main(int argc, char** argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建SerialSender节点
    auto serial_sender_node = std::make_shared<portable_serial_sender::SerialSender>();
    
    RCLCPP_INFO(serial_sender_node->get_logger(), "便携式串口发送节点已启动");
    
    // 创建用于发送的示例结构体
    MessageData msg{45.0, 30.0, 60.0, 20.0, true, 2.5, 90.0};
    
    // 创建用于接收的结构体和互斥锁
    FeedbackData feedback{};
    std::mutex feedback_mutex;
    
    // 创建/data_init话题的发布者
    auto data_init_pub = serial_sender_node->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/data_init", 10);
    
    // 启动接收线程
    std::thread receive_thread(receiveThread, serial_sender_node, std::ref(feedback), 
                              std::ref(feedback_mutex), data_init_pub);
    
    // 创建订阅，接收舵机角度数据
    auto servo_angle_sub = serial_sender_node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/servo_angle", 10, 
        [&msg](const std_msgs::msg::Float32MultiArray::SharedPtr servo_msg) {
            servoAngleCallback(servo_msg, msg);
        });
    
    // 周期性发送结构体 - 50ms (20Hz)
    rclcpp::Rate loop_rate(20); // 20赫兹 = 50毫秒周期
    
    while (rclcpp::ok())
    {
        // 发送结构体
        if (serial_sender_node->sendStruct(msg))
        {
            RCLCPP_INFO(serial_sender_node->get_logger(), 
                "结构体发送成功: servo1=%.2f, servo2=%.2f, servo3=%.2f, servo4=%.2f, "
                "is_grabing=%s, target_distance=%.2f, target_angle=%.2f", 
                msg.servo1, msg.servo2, msg.servo3, msg.servo4,
                msg.is_grabing ? "true" : "false", msg.target_distance, msg.target_angle);
            
            // 读取当前反馈数据（使用互斥锁保护）
            {
                std::lock_guard<std::mutex> lock(feedback_mutex);
                RCLCPP_INFO(serial_sender_node->get_logger(), "当前IMU Yaw: %.2f", feedback.imu_yaw);
            }
        }
        else
        {
            RCLCPP_ERROR(serial_sender_node->get_logger(), "发送结构体失败");
        }
        
        // 处理回调
        rclcpp::spin_some(serial_sender_node);
        
        loop_rate.sleep();
    }
    
    // 等待接收线程结束
    if(receive_thread.joinable())
    {
        receive_thread.join();
    }
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
} 
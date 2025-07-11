#include "serial_sender/serial_sender.hpp"
#include "serial_sender/struct_serializer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <navigation/msg/move_msg.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <queue>
#include <condition_variable>
#include <cmath> // 添加cmath头文件，用于M_PI常量

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

// 线程安全的数据包队列
class PacketQueue
{
public:
    void push(const std::vector<uint8_t>& packet)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(packet);
        }
        cv_.notify_one();
    }

    bool pop(std::vector<uint8_t>& packet, int timeout_ms = -1)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (timeout_ms < 0) {
            // 无超时，等待直到有数据
            cv_.wait(lock, [this] { return !queue_.empty(); });
        } else if (!cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                [this] { return !queue_.empty(); })) {
            // 超时
            return false;
        }
        
        packet = queue_.front();
        queue_.pop();
        return true;
    }

    bool empty()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    std::queue<std::vector<uint8_t>> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
};

// 数据接收线程函数 - 只负责读取串口数据并全部记录，不做任何处理
void receiveThread(const std::shared_ptr<portable_serial_sender::SerialSender>& serial_sender, 
                  PacketQueue& packet_queue)
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
                // 创建一个新的向量来存储接收到的数据
                std::vector<uint8_t> received_data(buffer.begin(), buffer.begin() + bytes_read);
            
                // 将读取到的数据直接放入队列，不进行任何处理
                packet_queue.push(received_data);
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(serial_sender->get_logger(), "读取串口数据错误: %s", e.what());
        }
    }
}

// 数据处理线程函数 - 负责判断数据帧的开始和结束，并解码数据
void processThread(const std::shared_ptr<portable_serial_sender::SerialSender>& serial_sender, 
                  PacketQueue& packet_queue,
                  FeedbackData& feedback,
                  std::mutex& feedback_mutex,
                  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_init_pub)
{
    std::vector<uint8_t> raw_data;
    std::vector<uint8_t> accumulated_buffer; // 用于累积数据
    
    while(rclcpp::ok())
    {
        // 从队列中获取原始数据，最多等待100ms
        if(packet_queue.pop(raw_data, 100))
        {
            // 将新接收的数据追加到累积缓冲区
            accumulated_buffer.insert(accumulated_buffer.end(), raw_data.begin(), raw_data.end());
            
            // 处理累积缓冲区中的所有完整数据包
            size_t search_start = 0;
            while(search_start < accumulated_buffer.size())
            {
                // 在剩余数据中寻找起始标记
                auto start_it = std::find(accumulated_buffer.begin() + search_start, accumulated_buffer.end(), 0xAA);
                if(start_it == accumulated_buffer.end())
                {
                    // 没有找到起始标记，丢弃所有数据并退出循环
                    accumulated_buffer.clear();
                    break;
                }
                
                // 计算起始标记位置的索引
                size_t start_pos = std::distance(accumulated_buffer.begin(), start_it);
                
                // 在起始标记之后寻找结束标记
                auto end_it = std::find(start_it + 1, accumulated_buffer.end(), 0xDD);
                if(end_it == accumulated_buffer.end())
                {
                    // 没有找到结束标记，保留从起始标记开始的所有数据
                    // 移动数据到缓冲区开始位置并退出循环
                    accumulated_buffer.erase(accumulated_buffer.begin(), start_it);
                    break;
                }
                
                // 计算结束标记位置的索引
                size_t end_pos = std::distance(accumulated_buffer.begin(), end_it);
                
                // 提取完整的数据包（包括起始和结束标记）
                std::vector<uint8_t> packet(accumulated_buffer.begin() + start_pos, accumulated_buffer.begin() + end_pos + 1);
                
                // 检查数据包大小是否符合预期
                if(packet.size() >= sizeof(FeedbackData) + 2) // 2 = 1(起始标记) + 1(结束标记)
                {
                    try
                    {
                        // 使用互斥锁保护共享数据
                        std::lock_guard<std::mutex> lock(feedback_mutex);
                        
                        // 将数据复制到结构体，跳过起始标记
                        std::memcpy(&feedback, &packet[1], sizeof(FeedbackData));

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
                    catch(const std::exception& e)
                    {
                        RCLCPP_ERROR(serial_sender->get_logger(), "处理数据包错误: %s", e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(serial_sender->get_logger(), "接收到的数据包大小不符合预期: %zu, 需要至少 %zu", 
                        packet.size(), sizeof(FeedbackData) + 2);
                }
                
                // 更新搜索起始位置为当前结束标记之后的位置
                search_start = end_pos + 1;
            }
        }
    }
}

// 舵机角度回调函数
void servoAngleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg, MessageData& message_data)
{
    // 检查消息数据长度是否足够
    if (msg->data.size() >= 5) {
        // 将弧度值转换为角度值 (radians * 180 / PI)
        message_data.servo1 = msg->data[0] * (180.0 / M_PI);
        message_data.servo2 = msg->data[1] * (180.0 / M_PI);
        message_data.servo3 = msg->data[2] * (180.0 / M_PI);
        message_data.servo4 = msg->data[3] * (180.0 / M_PI);
        message_data.is_grabing = (msg->data[4] > 0.5f) ? true : false;
    }
}

// 移动命令回调函数
void moveMsgCallback(const navigation::msg::MoveMsg::SharedPtr msg, MessageData& message_data)
{
    message_data.target_distance = msg->target_distance;
    message_data.target_angle = msg->target_angle;
}

int main(int argc, char** argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建SerialSender节点
    auto serial_sender_node = std::make_shared<portable_serial_sender::SerialSender>();
    
    RCLCPP_INFO(serial_sender_node->get_logger(), "便携式串口发送节点已启动");
    
    // 创建用于发送的示例结构体
    MessageData msg{0, 0, 0, 0, false, 0, 0};
    
    // 创建用于接收的结构体和互斥锁
    FeedbackData feedback{};
    std::mutex feedback_mutex;
    
    // 创建数据包队列
    PacketQueue packet_queue;
    
    // 创建/data_init话题的发布者
    auto data_init_pub = serial_sender_node->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/data_init", 10);
    
    // 启动接收线程
    std::thread receive_thread(receiveThread, serial_sender_node, std::ref(packet_queue));
    
    // 启动处理线程
    std::thread process_thread(processThread, serial_sender_node, std::ref(packet_queue),
                              std::ref(feedback), std::ref(feedback_mutex), data_init_pub);
    
    // 创建订阅，接收舵机角度数据
    auto servo_angle_sub = serial_sender_node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/servo_angle", 10, 
        [&msg](const std_msgs::msg::Float32MultiArray::SharedPtr servo_msg) {
            servoAngleCallback(servo_msg, msg);
        });
        
    // 创建订阅，接收移动命令数据
    auto move_msg_sub = serial_sender_node->create_subscription<navigation::msg::MoveMsg>(
        "/move_msg", 10,
        [&msg](const navigation::msg::MoveMsg::SharedPtr move_msg) {
            moveMsgCallback(move_msg, msg);
        });
    
    // 周期性发送结构体 - 50ms (20Hz)
    rclcpp::Rate loop_rate(20); // 20赫兹 = 50毫秒周期
    
    while (rclcpp::ok())
    {
        // 发送结构体
        serial_sender_node->sendStruct(msg);
        
        // 打印发送的舵机角度和夹爪状态信息
        RCLCPP_INFO(serial_sender_node->get_logger(), 
            "发送数据 - 舵机角度: [%.2f, %.2f, %.2f, %.2f], 夹爪: %s, 目标距离: %.2f, 目标角度: %.2f", 
            msg.servo1, msg.servo2, msg.servo3, msg.servo4, 
            msg.is_grabing ? "闭合" : "张开",
            msg.target_distance, msg.target_angle);
        
        // 处理回调
        rclcpp::spin_some(serial_sender_node);
        
        loop_rate.sleep();
    }
    
    // 等待接收线程和处理线程结束
    if(receive_thread.joinable())
    {
        receive_thread.join();
    }
    
    if(process_thread.joinable())
    {
        process_thread.join();
    }
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
} 
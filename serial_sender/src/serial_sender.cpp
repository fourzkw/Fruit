#include "serial_sender/serial_sender.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <thread> // 添加线程库以使用sleep_for

namespace portable_serial_sender
{

SerialSender::SerialSender()
: Node("serial_sender"),
  port_name_("/dev/ttyUSB0"), // 默认端口，可以通过参数设置
  baud_rate_(115200),         // 默认波特率，可以通过参数设置
  io_context_(std::make_shared<drivers::common::IoContext>(1)),
  is_connected_(false)
{
    // 声明参数
    this->declare_parameter<std::string>("port_name", port_name_);
    this->declare_parameter<int>("baud_rate", baud_rate_);
    
    // 获取参数
    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    
    RCLCPP_INFO(this->get_logger(), "串口: %s, 波特率: %d", port_name_.c_str(), baud_rate_);
    
    // 初始化类型映射
    initTypeMap();
    
    // 尝试连接串口
    if (connectToSerialPort())
    {
        RCLCPP_INFO(this->get_logger(), "已连接到串口");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "无法连接到串口");
    }
}

SerialSender::~SerialSender()
{
    if (is_connected_ && serial_driver_)
    {
        // 正确关闭串口
        serial_driver_->port()->close();
        RCLCPP_INFO(this->get_logger(), "已关闭串口");
    }
}

void SerialSender::initTypeMap()
{
    // 将C++类型映射到一个字节的标识符
    // 这些标识符在序列化数据时使用
    type_map_[std::type_index(typeid(int))] = 0x01;
    type_map_[std::type_index(typeid(float))] = 0x02;
    type_map_[std::type_index(typeid(double))] = 0x03;
    type_map_[std::type_index(typeid(bool))] = 0x04;
    type_map_[std::type_index(typeid(char))] = 0x05;
    // 根据需要添加更多类型
}

bool SerialSender::connectToSerialPort()
{
    try
    {
        // 创建串口配置
        serial_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
            baud_rate_,
            drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE,
            drivers::serial_driver::StopBits::ONE
        );

        // 创建串口驱动
        serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*io_context_);
        
        // 初始化和打开串口
        serial_driver_->init_port(port_name_, *serial_config_);
        serial_driver_->port()->open();
        
        if (serial_driver_->port()->is_open())
        {
            is_connected_ = true;
            return true;
        }
        else
        {
            is_connected_ = false;
            return false;
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "连接串口时发生异常: %s", e.what());
        is_connected_ = false;
        return false;
    }
}

std::vector<uint8_t> SerialSender::serializeInt(int value)
{
    std::vector<uint8_t> bytes(sizeof(int));
    std::memcpy(bytes.data(), &value, sizeof(int));
    return bytes;
}

std::vector<uint8_t> SerialSender::serializeFloat(float value)
{
    std::vector<uint8_t> bytes(sizeof(float));
    std::memcpy(bytes.data(), &value, sizeof(float));
    return bytes;
}

std::vector<uint8_t> SerialSender::serializeDouble(double value)
{
    std::vector<uint8_t> bytes(sizeof(double));
    std::memcpy(bytes.data(), &value, sizeof(double));
    return bytes;
}

std::vector<uint8_t> SerialSender::serializeBool(bool value)
{
    std::vector<uint8_t> bytes(1);
    bytes[0] = value ? 1 : 0;
    return bytes;
}

std::vector<uint8_t> SerialSender::serializeChar(char value)
{
    std::vector<uint8_t> bytes(1);
    bytes[0] = static_cast<uint8_t>(value);
    return bytes;
}

size_t SerialSender::readSerial(std::vector<uint8_t>& buffer)
{
    if (!is_connected_ && !connectToSerialPort())
    {
        RCLCPP_ERROR(this->get_logger(), "无法连接到串口");
        //等待500ms
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return 0;
    }

    try
    {
        // 尝试读取数据
        size_t bytes_read = serial_driver_->port()->receive(buffer);
        
        if (bytes_read > 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "从串口读取了 %zu 字节", bytes_read);
        }
        
        return bytes_read;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "读取串口数据时发生异常: %s", e.what());
        return 0;
    }
}

} // namespace portable_serial_sender 
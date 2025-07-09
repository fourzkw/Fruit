#ifndef SERIAL_SENDER_HPP
#define SERIAL_SENDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <typeindex>

namespace portable_serial_sender
{

class SerialSender : public rclcpp::Node
{
public:
    SerialSender();
    virtual ~SerialSender();

    /**
     * @brief 通过串口向下位机发送结构体数据
     * @param data 指向要发送的结构体的引用
     * @return 如果发送成功则返回true，否则返回false
     */
    template<typename T>
    bool sendStruct(const T& data);

    /**
     * @brief 从串口读取数据
     * @param buffer 用于存储接收数据的缓冲区
     * @return 读取的字节数，如果出错则返回0
     */
    size_t readSerial(std::vector<uint8_t>& buffer);

private:
    // 类型映射：将type_index映射到一个字节的标识符
    std::map<std::type_index, uint8_t> type_map_;
    
    // 串口参数
    std::string port_name_;
    int baud_rate_;
    std::shared_ptr<drivers::common::IoContext> io_context_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> serial_config_;
    bool is_connected_;

    // 初始化类型映射
    void initTypeMap();
    
    // 连接到串口
    bool connectToSerialPort();
    
    // 处理特定数据类型并返回序列化后的字节
    template<typename T>
    std::vector<uint8_t> serializeValue(const T& value);
    
    // 特定类型的序列化方法
    std::vector<uint8_t> serializeInt(int value);
    std::vector<uint8_t> serializeFloat(float value);
    std::vector<uint8_t> serializeDouble(double value);
    std::vector<uint8_t> serializeBool(bool value);
    std::vector<uint8_t> serializeChar(char value);
};

// 模板实现部分

template<typename T>
bool SerialSender::sendStruct(const T& data)
{
    if (!is_connected_ && !connectToSerialPort())
    {
        RCLCPP_ERROR(this->get_logger(), "无法连接到串口");
        return false;
    }

    std::vector<uint8_t> serialized_data;
    
    // 添加起始标记
    serialized_data.push_back(0xAA);  // 起始标记
    
    // 获取原始数据指针
    const uint8_t* raw_data = reinterpret_cast<const uint8_t*>(&data);
    size_t struct_size = sizeof(T);
    
    // 序列化结构体的每个字段
    // 将结构体视为字节序列直接添加
    for (size_t i = 0; i < struct_size; ++i)
    {
        serialized_data.push_back(raw_data[i]);
    }
    
    // 添加结束标记
    serialized_data.push_back(0xDD);  // 结束标记
    
    try
    {
        // 通过串口发送序列化数据
        size_t bytes_written = serial_driver_->port()->send(serialized_data);
        return bytes_written == serialized_data.size();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "发送数据错误: %s", e.what());
        return false;
    }
}

template<typename T>
std::vector<uint8_t> SerialSender::serializeValue(const T& value)
{
    // 获取类型标识符
    auto type_it = type_map_.find(std::type_index(typeid(T)));
    uint8_t type_id = (type_it != type_map_.end()) ? type_it->second : 0xFF; // 0xFF表示未知类型
    
    std::vector<uint8_t> result;
    result.push_back(type_id); // 添加类型标识符
    
    // 根据值的类型进行序列化
    if constexpr (std::is_same_v<T, int>)
    {
        auto value_bytes = serializeInt(value);
        result.insert(result.end(), value_bytes.begin(), value_bytes.end());
    }
    else if constexpr (std::is_same_v<T, float>)
    {
        auto value_bytes = serializeFloat(value);
        result.insert(result.end(), value_bytes.begin(), value_bytes.end());
    }
    else if constexpr (std::is_same_v<T, double>)
    {
        auto value_bytes = serializeDouble(value);
        result.insert(result.end(), value_bytes.begin(), value_bytes.end());
    }
    else if constexpr (std::is_same_v<T, bool>)
    {
        auto value_bytes = serializeBool(value);
        result.insert(result.end(), value_bytes.begin(), value_bytes.end());
    }
    else if constexpr (std::is_same_v<T, char>)
    {
        auto value_bytes = serializeChar(value);
        result.insert(result.end(), value_bytes.begin(), value_bytes.end());
    }
    else
    {
        // 对于不支持的类型，我们直接复制原始字节
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
        result.insert(result.end(), bytes, bytes + sizeof(T));
    }
    
    return result;
}

} // namespace portable_serial_sender

#endif // SERIAL_SENDER_HPP 
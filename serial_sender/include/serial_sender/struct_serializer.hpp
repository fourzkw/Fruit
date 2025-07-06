#ifndef STRUCT_SERIALIZER_HPP
#define STRUCT_SERIALIZER_HPP

#include <vector>
#include <cstdint>
#include <string>
#include <typeinfo>
#include <map>
#include <typeindex>

namespace portable_serial_sender
{

// 不同数据类型的枚举
enum class DataType : uint8_t
{
    INT8    = 0x01,
    UINT8   = 0x02,
    INT16   = 0x03,
    UINT16  = 0x04,
    INT32   = 0x05,
    UINT32  = 0x06,
    FLOAT   = 0x07,
    DOUBLE  = 0x08,
    BOOL    = 0x09,
    CHAR    = 0x0A,
    UNKNOWN = 0xFF
};

// 描述结构体中字段的结构体
struct FieldDescriptor
{
    std::string name;
    DataType type;
    size_t offset;
    size_t size;
};

// 结构化序列化类
class StructSerializer
{
public:
    StructSerializer();
    virtual ~StructSerializer() = default;

    // 注册带有字段描述的结构体类型
    template<typename T>
    void registerStruct(const std::string& struct_name, const std::vector<FieldDescriptor>& fields);

    // 将结构体序列化为二进制数据
    template<typename T>
    std::vector<uint8_t> serialize(const T& data, const std::string& struct_name);

    // 将C++类型转换为DataType枚举
    DataType getDataTypeFromTypeIndex(const std::type_index& index);

private:
    // 按名称存储结构体定义
    std::map<std::string, std::vector<FieldDescriptor>> struct_definitions_;
    
    // C++内置类型的映射
    std::map<std::type_index, DataType> type_map_;
    
    // 初始化类型映射
    void initTypeMap();
};

// 模板实现
template<typename T>
void StructSerializer::registerStruct(const std::string& struct_name, const std::vector<FieldDescriptor>& fields)
{
    struct_definitions_[struct_name] = fields;
}

template<typename T>
std::vector<uint8_t> StructSerializer::serialize(const T& data, const std::string& struct_name)
{
    std::vector<uint8_t> result;
    
    // 查找结构体定义
    auto it = struct_definitions_.find(struct_name);
    if (it == struct_definitions_.end())
    {
        // 结构体未注册，返回空向量
        return result;
    }
    
    const auto& fields = it->second;
    const uint8_t* raw_data = reinterpret_cast<const uint8_t*>(&data);
    
    // 添加头部字节（可自定义）
    result.push_back(0xAA); // 起始标记
    result.push_back(0xBB); // 起始标记
    
    // 添加字段数量
    result.push_back(static_cast<uint8_t>(fields.size()));
    
    // 序列化每个字段
    for (const auto& field : fields)
    {
        // 添加字段类型
        result.push_back(static_cast<uint8_t>(field.type));
        
        // 添加字段值大小
        result.push_back(static_cast<uint8_t>(field.size));
        
        // 添加字段值
        const uint8_t* field_data = raw_data + field.offset;
        result.insert(result.end(), field_data, field_data + field.size);
    }
    
    // 添加结束标记
    result.push_back(0xCC); // 结束标记
    result.push_back(0xDD); // 结束标记
    
    return result;
}

} // namespace portable_serial_sender

#endif // STRUCT_SERIALIZER_HPP 
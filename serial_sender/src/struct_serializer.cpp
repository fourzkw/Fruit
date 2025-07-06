#include "serial_sender/struct_serializer.hpp"

namespace portable_serial_sender
{

StructSerializer::StructSerializer()
{
    initTypeMap();
}

void StructSerializer::initTypeMap()
{
    // 初始化C++类型到DataType枚举的映射
    type_map_[std::type_index(typeid(int8_t))] = DataType::INT8;
    type_map_[std::type_index(typeid(uint8_t))] = DataType::UINT8;
    type_map_[std::type_index(typeid(int16_t))] = DataType::INT16;
    type_map_[std::type_index(typeid(uint16_t))] = DataType::UINT16;
    type_map_[std::type_index(typeid(int32_t))] = DataType::INT32;
    type_map_[std::type_index(typeid(uint32_t))] = DataType::UINT32;
    type_map_[std::type_index(typeid(float))] = DataType::FLOAT;
    type_map_[std::type_index(typeid(double))] = DataType::DOUBLE;
    type_map_[std::type_index(typeid(bool))] = DataType::BOOL;
    type_map_[std::type_index(typeid(char))] = DataType::CHAR;
}

DataType StructSerializer::getDataTypeFromTypeIndex(const std::type_index& index)
{
    auto it = type_map_.find(index);
    if (it != type_map_.end())
    {
        return it->second;
    }
    return DataType::UNKNOWN;
}

} // namespace portable_serial_sender 
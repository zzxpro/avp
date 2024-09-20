#include "value_conversion_tables.h"

namespace AVP
{
    
namespace mapping
{
    
constexpr uint16 kUpdateMarker = 1u << 15; // 32768

/**
 * 将输入的值转换为浮点数，范围[lower_bound, upper_bound]。如果值为unknown_value，则直接返回unknown_result
 */
float SlowValueToBoundedFloat(const uint16 value, 
                              const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) 
{
    if (value == unknown_value) return unknown_result;
    // 32766.f最大有效值，uint16范围 0 ~ 65535，kScale是将【0，32766】范围内整数映射到[lower_bound, upper_bound]的缩放因子
    const float kScale = (upper_bound - lower_bound) / 32766.f;
    // value*kScale将value映射到新的范围 加偏置，使结果保持在lower_bound 和 upper_bound之间
    return value * kScale + (lower_bound - kScale);
}

/**
 * 预计算从0 到 32767所有值的对应的浮点数值，并返回一个指向结果的智能指针
 */
std::shared_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) 
{
    auto result = std::make_shared<std::vector<float>>();
    // num_values = 65536 = 65535 + 1
    size_t num_values = std::numeric_limits<uint16>::max() + 1; 
    // 申请空间
    result->reserve(num_values);

    // 将[0, 1~32767]映射成[0.9, 0.1~0.9]
    // vector的个数为65536, 所以存的是2遍[0-32767]的映射
    for (size_t value = 0; value != num_values; ++value) 
    {
        result->push_back(SlowValueToBoundedFloat(
            static_cast<uint16>(value) & ~kUpdateMarker, // 取右边15位的数据, 0-32767 1000000000000000
            unknown_value,
            unknown_result, lower_bound, upper_bound));
    }
    return result;
}


/**
 * 根据输入的参数获取转换表
 */
const std::vector<float>* ValueConversionTables::GetConversionTable(
    float unknown_result, float lower_bound, float upper_bound) 
{
    // 将bounds作为key
    std::tuple<float, float, float> bounds =
        std::make_tuple(unknown_result, lower_bound, upper_bound);
    auto lookup_table_iterator = bounds_to_lookup_table_.find(bounds);

    // 如果没有bounds这个key就新建
    if (lookup_table_iterator == bounds_to_lookup_table_.end()) 
    {
        // 新建转换表
        auto insertion_result = bounds_to_lookup_table_.emplace(
            bounds, PrecomputeValueToBoundedFloat(0, unknown_result, lower_bound,
                                                upper_bound));
        return insertion_result.first->second.get();
    } 
    // 如果存在就直接返回原始指针
    else 
    {
        return lookup_table_iterator->second.get();
    }
}


} // namespace mapping


} // namespace AVP

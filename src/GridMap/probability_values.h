#ifndef PROBABILITY_VALUES_H
#define PROBABILITY_VALUES_H

#include "port.h"
#include "math.h"
#include <vector>
#include <memory>

namespace AVP
{
    
namespace mapping
{
    

namespace {
// 为了避免浮点运算, 将[0~0.9]的浮点数转成[1~32767]之间的值
inline uint16 BoundedFloatToValue(const float float_value,
                                  const float lower_bound,
                                  const float upper_bound) {
  const int value =
      common::RoundToInt(
          (common::Clamp(float_value, lower_bound, upper_bound) - lower_bound) *
          (32766.f / (upper_bound - lower_bound))) +
      1;
  return value;
}

}  // namespace

// Odd = p(a)/(1 - p(a)) 为成功事件的概率比失败事件的概率
// 通过概率计算Odd值 论文里的 odds(p)函数
inline float Odds(float probability) {
  return probability / (1.f - probability);
}

// 通过Odd值计算概率值 论文里的 odds^-1 函数
inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

// probability与CorrespondenceCost的关系,CorrespondenceCost代表free的概率
// 实际上是对输入概率值取反，意味着如果输入描述的是栅格单元的占用概率，则实际存储的是栅格单元的空闲概率
inline float ProbabilityToCorrespondenceCost(const float probability) {
  return 1.f - probability;
}

inline float CorrespondenceCostToProbability(const float correspondence_cost) {
  return 1.f - correspondence_cost;
}

// 描述了栅格元素的最小值和最大值
constexpr float kMinProbability = 0.1f;                         // 0.1
constexpr float kMaxProbability = 1.f - kMinProbability;        // 0.9
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability; // 0.1
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability; // 0.9

// Clamps probability to be in the range [kMinProbability, kMaxProbability].
// 对数据进行上下界的限定
inline float ClampProbability(const float probability) {
  return common::Clamp(probability, kMinProbability, kMaxProbability);
}

// Clamps correspondece cost to be in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
// 对数据进行上下界的限定
inline float ClampCorrespondenceCost(const float correspondence_cost) {
  return common::Clamp(correspondence_cost, kMinCorrespondenceCost,
                       kMaxCorrespondenceCost);
}

constexpr uint16 kUnknownProbabilityValue = 0; // 0
constexpr uint16 kUnknownCorrespondenceValue = kUnknownProbabilityValue; // 0
// 为已更新标志量，防止miss和hit重复对地图更新，其值为32768，为uint16的最高位，其操作方式是，更新时加上，更新完后减去
constexpr uint16 kUpdateMarker = 1u << 15; 

// Converts a correspondence_cost to a uint16 in the [1, 32767] range.
// 将浮点数correspondence_cost转成[1, 32767]范围内的 uint16 整数
// 并将输入从区间[kMinCorrespondenceCost, kMaxCorrespondenceCost]映射到[1, 32767]
inline uint16 CorrespondenceCostToValue(const float correspondence_cost) {
  return BoundedFloatToValue(correspondence_cost, kMinCorrespondenceCost,
                             kMaxCorrespondenceCost);
}

// Converts a probability to a uint16 in the [1, 32767] range.
// 将浮点数probability转成[1, 32767]范围内的 uint16 整数
inline uint16 ProbabilityToValue(const float probability) {
  return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
}


// c++11: extern c风格
extern const std::vector<float>* const kValueToProbability;
extern const std::vector<float>* const kValueToCorrespondenceCost;

// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}

// Converts a uint16 (which may or may not have the update marker set) to a
// correspondence cost in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
inline float ValueToCorrespondenceCost(const uint16 value) {
  return (*kValueToCorrespondenceCost)[value];
}

// 测试用的函数
inline uint16 ProbabilityValueToCorrespondenceCostValue(
    uint16 probability_value) {
  if (probability_value == kUnknownProbabilityValue) {
    return kUnknownCorrespondenceValue;
  }
  bool update_carry = false;
  if (probability_value > kUpdateMarker) {
    probability_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16 result = CorrespondenceCostToValue(
      ProbabilityToCorrespondenceCost(ValueToProbability(probability_value)));
  if (update_carry) result += kUpdateMarker;
  return result;
}

// 测试用的函数
inline uint16 CorrespondenceCostValueToProbabilityValue(
    uint16 correspondence_cost_value) {
  if (correspondence_cost_value == kUnknownCorrespondenceValue)
    return kUnknownProbabilityValue;
  bool update_carry = false;
  if (correspondence_cost_value > kUpdateMarker) {
    correspondence_cost_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16 result = ProbabilityToValue(CorrespondenceCostToProbability(
      ValueToCorrespondenceCost(correspondence_cost_value)));
  if (update_carry) result += kUpdateMarker;
  return result;
}

std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(float odds);

} // namespace mapping
} // namespace AVP


#endif
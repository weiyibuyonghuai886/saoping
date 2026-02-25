#include "utilities.h"

namespace legged {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 移动平均滤波器类实现
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MovingAverageFilter::MovingAverageFilter(size_t sample_count)
    : sample_count_(sample_count), sum_(0.0) {
}

double MovingAverageFilter::applyFilter(double new_value) {
    // 添加新样本
    samples_.push_back(new_value);
    sum_ += new_value;

    // 如果超过了样本数量限制，移除最旧的样本
    if (samples_.size() > sample_count_) {
        sum_ -= samples_.front();
        samples_.pop_front();
    }

    // 返回平均值
    return sum_ / static_cast<double>(samples_.size());
}

void MovingAverageFilter::reset() {
    samples_.clear();
    sum_ = 0.0;
}

double MovingAverageFilter::getCurrentValue() const {
    if (samples_.empty()) {
        return 0.0;
    }
    return sum_ / static_cast<double>(samples_.size());
}

// 便捷函数实现
double applyMovingAverageFilter(double value, MovingAverageFilter& filter) {
    return filter.applyFilter(value);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 便捷函数实现
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std_msgs::msg::Float64MultiArray createFloat64MultiArrayFromVector(const vector_t& data)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(data.size());
  // copy data from eigen vector to std vector
  vector_t::Map(&msg.data[0], data.size()) = data;
  return msg;
}

}
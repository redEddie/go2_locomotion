#pragma once

#include <deque>

namespace leg_odometry_ros2
{

class MovingWindowFilter
{
public:
  explicit MovingWindowFilter(std::size_t window_size) : window_size_(window_size) {}

  double update(double value)
  {
    if (window_size_ == 0)
    {
      return value;
    }
    buffer_.push_back(value);
    sum_ += value;
    if (buffer_.size() > window_size_)
    {
      sum_ -= buffer_.front();
      buffer_.pop_front();
    }
    return sum_ / static_cast<double>(buffer_.size());
  }

  void reset()
  {
    buffer_.clear();
    sum_ = 0.0;
  }

private:
  std::size_t window_size_ = 1;
  std::deque<double> buffer_;
  double sum_ = 0.0;
};

}  // namespace leg_odometry_ros2

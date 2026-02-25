/**
 * @file time.h
 * @brief Provides a ROS-agnostic Time class utilizing chrono-based time
 *
 * This class also provides operators to compute differences between time instances
 * and compare two times
 */
#pragma once

#include <chrono>

namespace polaris
{
class Time
{
public:
    using Clock     = std::chrono::system_clock;
    using TimePoint = Clock::time_point;

    explicit Time() : Time(TimePoint{})
    {}

    explicit Time(const TimePoint& tp) : tp_{tp}
    {}

    const TimePoint& timePoint() const { return tp_; }

    // Get seconds elapsed (floating-point) between the two times
    friend double operator-(const Time& end,
                            const Time& start)
    {
        using Duration = std::chrono::duration<double>;
        return std::chrono::duration_cast<Duration>(
                   end.tp_ - start.tp_).count();
    }

    // Comparison operators
    friend bool operator==(const Time& lhs,
                           const Time& rhs)
    {
        return lhs.tp_ == rhs.tp_;
    }

    friend bool operator!=(const Time& lhs,
                           const Time& rhs)
    {
        return !(lhs == rhs);
    }

    friend bool operator<(const Time& lhs,
                          const Time& rhs)
    {
        return lhs.tp_ < rhs.tp_;
    }

    friend bool operator<=(const Time& lhs,
                           const Time& rhs)
    {
        return !(rhs < lhs);
    }

    friend bool operator>(const Time& lhs,
                          const Time& rhs)
    {
        return rhs < lhs;
    }

    friend bool operator>=(const Time& lhs,
                           const Time& rhs)
    {
        return !(lhs < rhs);
    }

private:
    TimePoint tp_;
};
}
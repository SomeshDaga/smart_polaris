#pragma once

#include <ros/time.h>
#include <gem_state_manager/time.h>

using namespace polaris;

static Time fromRosTime(const ros::Time& t)
{
    return Time(Time::TimePoint{} +
            std::chrono::seconds(t.sec) +
            std::chrono::nanoseconds(t.nsec));
}
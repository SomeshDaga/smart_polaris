cmake_minimum_required(VERSION 3.0.2)
project(gem_state_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
    BatteryLevelStamped.msg
    EstopStamped.msg
    GpsAccuracyStamped.msg
    SignalStrengthStamped.msg
    SystemStateStamped.msg
    TemperatureStamped.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS Boost)
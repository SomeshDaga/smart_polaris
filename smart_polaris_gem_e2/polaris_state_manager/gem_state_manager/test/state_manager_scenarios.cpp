/**
 * @file state_manager_scenarios.cpp
 * @brief Scenarios testing for state_manager_node.
 *
 * Each test drives a scenario by publishing to state_manager inputs and
 * asserts expected system states
 */

#include <gem_state_msgs/BatteryLevelStamped.h>
#include <gem_state_msgs/EstopStamped.h>
#include <gem_state_msgs/GpsAccuracyStamped.h>
#include <gem_state_msgs/SignalStrengthStamped.h>
#include <gem_state_msgs/SystemStateStamped.h>
#include <gem_state_msgs/TemperatureStamped.h>
#include <gem_state_msgs/TaskPlannerStatus.h>

#include <ros/message_traits.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

namespace
{

// Publish rate for messages when continuous publishing
constexpr double PUB_RATE = 10.0;
constexpr double STATE_MANAGER_NODE_INIT_TIMEOUT = 60.0;

class StateManagerScenariosTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    nh_ = ros::NodeHandle();

    system_state_received_ = false;
    last_system_state_.error_code = 0;
    last_system_state_.state = 0;

    battery_pub_ = nh_.advertise<gem_state_msgs::BatteryLevelStamped>("/battery_level", 10);
    temperature_pub_ = nh_.advertise<gem_state_msgs::TemperatureStamped>("/temperature", 10);
    gps_pub_ = nh_.advertise<gem_state_msgs::GpsAccuracyStamped>("/gps_accuracy", 10);
    signal_pub_ = nh_.advertise<gem_state_msgs::SignalStrengthStamped>("/signal_strength", 10);
    estop_pub_ = nh_.advertise<gem_state_msgs::EstopStamped>("/estop", 10);
    system_state_sub_ = nh_.subscribe("/system_state", 10, &StateManagerScenariosTest::systemStateCb, this);
  }

  void systemStateCb(const gem_state_msgs::SystemStateStamped::ConstPtr& msg)
  {
    last_system_state_ = *msg;
    system_state_received_ = true;
  }

  void waitForFirstSystemState()
  {
    system_state_received_ = false;
    ros::Time start = ros::Time::now();
    while (!system_state_received_)
    {
      ros::spinOnce();
      if ((ros::Time::now() - start).toSec() > STATE_MANAGER_NODE_INIT_TIMEOUT)
        break;
      ros::Duration(0.05).sleep();
    }
    ASSERT_TRUE(system_state_received_) << "No system_state received within timeout";
  }

  void publishGoodState()
  {
    ros::Rate publishRate(PUB_RATE);
    ros::Time end = ros::Time::now() + ros::Duration(1.0);
  
    gem_state_msgs::BatteryLevelStamped batteryMsg;
    gem_state_msgs::TemperatureStamped tempMsg;
    gem_state_msgs::GpsAccuracyStamped gpsMsg;
    gem_state_msgs::SignalStrengthStamped signalMsg;
    gem_state_msgs::EstopStamped estopMsg;


    while (ros::Time::now() < end)
    {
      publishOnce(battery_pub_, batteryMsg, batteryMsg.battery_level, 1.f);
      publishOnce(temperature_pub_, tempMsg, tempMsg.temperature, 45.f);
      publishOnce(gps_pub_, gpsMsg, gpsMsg.accuracy, 100.f);
      publishOnce(signal_pub_, signalMsg, signalMsg.strength, static_cast<uint8_t>(gem_state_msgs::SignalStrengthStamped::CONNECTED));
      publishOnce(estop_pub_, estopMsg, estopMsg.enabled, static_cast<uint8_t>(false));

      ros::spinOnce();
      publishRate.sleep();
    }
  }

  /**
   * @brief Wait for system state change to/from given state and return the new state
   * @param state State to transition to/from
   * @param timeout Timeout for observed state transition
   * @param shouldChangeTo true if require a state transition into `state`, otherwise require a state transition out of `state`
   * @return shared_ptr to updated SystemStateStamped message (nullptr if state transition is not seen)
   */
  std::shared_ptr<gem_state_msgs::SystemStateStamped> waitForStateUpdate(uint8_t state, const ros::Duration& timeout, const bool shouldChangeTo = true)
  {
    ros::Time deadline = ros::Time::now() + timeout;
    while (ros::Time::now() < deadline)
    {
      ros::spinOnce();
      if (last_system_state_.state == state ^ !shouldChangeTo)
      {
        return std::make_shared<gem_state_msgs::SystemStateStamped>(last_system_state_);
      }
      ros::Duration(timeout.toSec() / 10.).sleep();
    }
    return nullptr;
  }

  template <typename MessageT>
  typename std::enable_if_t<ros::message_traits::HasHeader<MessageT>::value>
  setHeader(MessageT& msg)
  {
      ros::message_traits::TimeStamp<MessageT>::value(msg) = ros::Time::now();
  }

  template <typename MessageT>
  typename std::enable_if_t<!ros::message_traits::HasHeader<MessageT>::value>
  setHeader(MessageT&)
  {
      // Do nothing if the message does not have a header
  }

  template <typename MessageT, typename ValueT>
  typename std::enable_if_t<std::is_arithmetic<ValueT>::value>
  publishLinear(ros::Publisher& publisher, MessageT& msg, ValueT& field, const ValueT start, const ValueT end, const ros::Duration& period)
  {
    const int32_t nSteps = static_cast<int32_t>(std::ceil(period.toSec() * PUB_RATE));
    ros::Rate loopRate(PUB_RATE);
    for (int32_t step = 0; step < nSteps; step++)
    {
      setHeader(msg);
      field = start + (end - start) * static_cast<float>(step) / static_cast<float>(nSteps - 1);
      publisher.publish(msg);
      loopRate.sleep();
      ros::spinOnce();
    }
  }

  template <typename MessageT, typename ValueT>
  void publishOnce(ros::Publisher& publisher, MessageT& msg, ValueT& field, const ValueT value)
  {
    setHeader(msg);
    field = value;
    publisher.publish(msg);
    ros::spinOnce();    
  }

  template <typename MessageT, typename ValueT>
  void publishRepeat(ros::Publisher& publisher, MessageT& msg, ValueT& field, const ValueT value, const ros::Duration& period)
  {
    const ros::Time startTime = ros::Time::now();
    ros::Rate loopRate(PUB_RATE);

    setHeader(msg);
    field = value;
    while (ros::Time::now() < startTime + period)
    {
      publisher.publish(msg);
      loopRate.sleep();
      ros::spinOnce();
    }
  }

  template <typename MessageT, typename ValueT>
  void publishStep(ros::Publisher& publisher, MessageT& msg, ValueT& field, const ValueT start, const ValueT end, const ros::Duration& period)
  {
    publishOnce(publisher, msg, field, start);
    period.sleep();
    publishOnce(publisher, msg, field, end);
  }

  ros::NodeHandle nh_;
  ros::Publisher battery_pub_;
  ros::Publisher temperature_pub_;
  ros::Publisher gps_pub_;
  ros::Publisher signal_pub_;
  ros::Publisher estop_pub_;
  ros::Subscriber system_state_sub_;
  volatile bool system_state_received_;
  gem_state_msgs::SystemStateStamped last_system_state_;
};

// 1. Battery Failure: 100% -> 51% over 30s, then 49%. Expect ERROR at 50%.
TEST_F(StateManagerScenariosTest, BatteryFailure)
{
  using MessageT = gem_state_msgs::BatteryLevelStamped;
  publishGoodState();
  waitForFirstSystemState();

  MessageT msg;
  std::shared_ptr<gem_state_msgs::SystemStateStamped> updatedStatePtr = nullptr;

  // Publish battery level 100% -> 51% over 30s
  publishLinear(battery_pub_, msg, msg.battery_level, 1.0f, 0.51f, ros::Duration(1.0));

  // Check no error is seen within the timeout
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_FALSE(updatedStatePtr);

  // Publish battery level 49%
  publishOnce(battery_pub_, msg, msg.battery_level, 0.49f);

  // System should transition to an error state
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_TRUE(updatedStatePtr);
  // Ensure that system error is due to low battery
  EXPECT_TRUE(updatedStatePtr->error_code == gem_state_msgs::SystemStateStamped::ERROR_BATTERY_LOW);
}

// 2. Temperature Spike: 30°C -> 55°C over 30s, then 60°C. ERROR at 55°C.
TEST_F(StateManagerScenariosTest, TemperatureSpike)
{
  using MessageT = gem_state_msgs::TemperatureStamped;
  publishGoodState();
  waitForFirstSystemState();

  MessageT msg;
  std::shared_ptr<gem_state_msgs::SystemStateStamped> updatedStatePtr = nullptr;

  // Publish temperature 30°C -> 55°C over 30s
  publishLinear(temperature_pub_, msg, msg.temperature, 30.f, 55.f, ros::Duration(1.0));

  // Check no error is seen within the timeout
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_FALSE(updatedStatePtr);

  // Publish temperature 60°C
  publishOnce(temperature_pub_, msg, msg.temperature, 60.f);

  // System should transition to an error state
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_TRUE(updatedStatePtr);
  // Ensure that system error is due to high temperature
  EXPECT_TRUE(updatedStatePtr->error_code == gem_state_msgs::SystemStateStamped::ERROR_TEMPERATURE_HIGH);
}

// // 3. GPS Fluctuation: good initially, then >200 mm for 20s. ERROR if >200 mm >= 15s.
TEST_F(StateManagerScenariosTest, GpsFluctuation)
{
  using MessageT = gem_state_msgs::GpsAccuracyStamped;
  publishGoodState();
  waitForFirstSystemState();

  MessageT msg;
  std::shared_ptr<gem_state_msgs::SystemStateStamped> updatedStatePtr = nullptr;

  // Publish good GPS accuracy (< 200mm for 20 seconds)
  publishRepeat(gps_pub_, msg, msg.accuracy, 100.f, ros::Duration(20.0));

  // Check no error is seen within the timeout
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_FALSE(updatedStatePtr);

  // Publish bad GPS accuracy for 20 seconds
  publishRepeat(gps_pub_, msg, msg.accuracy, 201.f, ros::Duration(20.0));

  // System should transition to an error state
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_TRUE(updatedStatePtr);
  // Ensure that system error is due to bad gps accuracy
  EXPECT_TRUE(updatedStatePtr->error_code == gem_state_msgs::SystemStateStamped::ERROR_GPS_LOST);
}

// // 4. Network: Connected -> Low 20s -> Connected -> Not Connected 10s. Expect ERROR (signal low and signal lost).
TEST_F(StateManagerScenariosTest, NetworkSignalFluctuation)
{
  using MessageT = gem_state_msgs::SignalStrengthStamped;
  publishGoodState();
  waitForFirstSystemState();

  MessageT msg;
  std::shared_ptr<gem_state_msgs::SystemStateStamped> updatedStatePtr = nullptr;

  // Publish Connected state (for 1 second)
  publishRepeat(signal_pub_, msg, msg.strength, static_cast<uint8_t>(MessageT::CONNECTED), ros::Duration(1.0));

  // Check no error is seen within the timeout
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_FALSE(updatedStatePtr);

  // Publish Low signal state for 20 seconds
  // NOTE (someshdaga): Possible typo in assignment mentioning low signal for 10 seconds (but error condition is 20 seconds)
  //                    Switching periods for low and not connected signals ie. 20s and 10s respectively
  publishRepeat(signal_pub_, msg, msg.strength, static_cast<uint8_t>(MessageT::LOW_SIGNAL), ros::Duration(20.0));
  // System should transition to an error state
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_TRUE(updatedStatePtr);
  // Ensure that system error is due to low signal
  EXPECT_TRUE(updatedStatePtr->error_code == gem_state_msgs::SystemStateStamped::ERROR_SIGNAL_LOW);

  // Publish Connected state
  publishOnce(signal_pub_, msg, msg.strength, static_cast<uint8_t>(MessageT::CONNECTED));

  // Check recovery from error state within timeout
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), false);
  ASSERT_TRUE(updatedStatePtr);

  // Publish NOT_CONNECTED state for 10 seconds
  publishRepeat(signal_pub_, msg, msg.strength, static_cast<uint8_t>(MessageT::NOT_CONNECTED), ros::Duration(10.0));
  // System should transition to an error state
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_TRUE(updatedStatePtr);
  // Ensure that system error is due to low signal
  EXPECT_TRUE(updatedStatePtr->error_code == gem_state_msgs::SystemStateStamped::ERROR_SIGNAL_LOST);
}

// 5. Emergency Stop: on true, immediate ERROR.
TEST_F(StateManagerScenariosTest, EmergencyStop)
{
  using MessageT = gem_state_msgs::EstopStamped;
  publishGoodState();
  waitForFirstSystemState();

  MessageT msg;
  std::shared_ptr<gem_state_msgs::SystemStateStamped> updatedStatePtr = nullptr;

  // Publish estop disabled state
  publishOnce(estop_pub_, msg, msg.enabled, static_cast<uint8_t>(false));

  // Check no error is seen within the timeout
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_FALSE(updatedStatePtr);

  // Publish estop enabled state
  publishOnce(estop_pub_, msg, msg.enabled, static_cast<uint8_t>(true));

  // System should transition to an error state
  updatedStatePtr = waitForStateUpdate(gem_state_msgs::SystemStateStamped::STATE_ERROR, ros::Duration(0.2), true);
  ASSERT_TRUE(updatedStatePtr);
  // Ensure that system error is due to low signal
  EXPECT_TRUE(updatedStatePtr->error_code == gem_state_msgs::SystemStateStamped::ERROR_ESTOP);
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_manager_scenarios_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <gem_state_manager/state_manager.h>
#include <gem_state_manager/ros_time_helpers.h>

#include <gem_state_msgs/BatteryLevelStamped.h>
#include <gem_state_msgs/EstopStamped.h>
#include <gem_state_msgs/GpsAccuracyStamped.h>
#include <gem_state_msgs/SignalStrengthStamped.h>
#include <gem_state_msgs/TemperatureStamped.h>
#include <gem_state_msgs/SystemStateStamped.h>

#include <ros/ros.h>

namespace polaris
{
class StateManagerNode
{
public:
    StateManagerNode() :
        nh_{},
        stateManager_{}
    {
        systemStatePub_ = nh_.advertise<gem_state_msgs::SystemStateStamped>("system_state", 10);

        batteryLevelSub_ = nh_.subscribe("battery_level", 10, &StateManagerNode::batteryLevelCb, this);
        estopSub_ = nh_.subscribe("estop", 10, &StateManagerNode::estopCb, this);
        gpsAccuracySub_ = nh_.subscribe("gps_accuracy", 10, &StateManagerNode::gpsAccuracyCb, this);
        signalStrengthSub_ = nh_.subscribe("signal_strength", 10, &StateManagerNode::signalStrengthCb, this);
        temperatureSub_ = nh_.subscribe("temperature", 10, &StateManagerNode::temperatureCb, this);
    }

    void batteryLevelCb(const gem_state_msgs::BatteryLevelStamped::ConstPtr& msg)
    {
        stateManager_.updateBatteryLevel(fromRosTime(msg->header.stamp), msg->battery_level);
    }

    void estopCb(const gem_state_msgs::EstopStamped::ConstPtr& msg)
    {
        stateManager_.updateEmergencyStop(fromRosTime(msg->header.stamp), msg->enabled);
    }

    void gpsAccuracyCb(const gem_state_msgs::GpsAccuracyStamped::ConstPtr& msg)
    {
        stateManager_.updateGpsAccuracy(fromRosTime(msg->header.stamp), msg->accuracy);
    }

    void signalStrengthCb(const gem_state_msgs::SignalStrengthStamped::ConstPtr& msg)
    {
        SignalStrength strength;
        switch (msg->strength)
        {
        case gem_state_msgs::SignalStrengthStamped::NOT_CONNECTED:
            strength = SignalStrength::NOT_CONNECTED;
            break;
        case gem_state_msgs::SignalStrengthStamped::CONNECTED:
            strength = SignalStrength::CONNECTED;
            break;
        case gem_state_msgs::SignalStrengthStamped::LOW_SIGNAL:
            strength = SignalStrength::LOW;
        };
        stateManager_.updateSignalStrength(fromRosTime(msg->header.stamp), strength);
    }

    void temperatureCb(const gem_state_msgs::TemperatureStamped::ConstPtr& msg)
    {
        stateManager_.updateTemperature(fromRosTime(msg->header.stamp), msg->temperature);
    }

    void publishSystemState(const Time& now)
    {
        gem_state_msgs::SystemStateStamped systemStateMsg;
        systemStateMsg.header.stamp = ros::Time::now();

        if (stateManager_.isBatteryLow())
        { systemStateMsg.error_code |= gem_state_msgs::SystemStateStamped::ERROR_BATTERY_LOW; }

        if (stateManager_.isEstopEnabled())
        { systemStateMsg.error_code |= gem_state_msgs::SystemStateStamped::ERROR_ESTOP; }

        if (stateManager_.isGpsBadPersistent(now))
        { systemStateMsg.error_code |= gem_state_msgs::SystemStateStamped::ERROR_GPS_LOST; }

        if (stateManager_.isSignalNotConnectedPersistent(now))
        { systemStateMsg.error_code |= gem_state_msgs::SystemStateStamped::ERROR_SIGNAL_LOST; }

        if (stateManager_.isSignalLowPersistent(now))
        { systemStateMsg.error_code |= gem_state_msgs::SystemStateStamped::ERROR_SIGNAL_LOW; }

        if (stateManager_.isTemperatureHigh())
        { systemStateMsg.error_code |= gem_state_msgs::SystemStateStamped::ERROR_TEMPERATURE_HIGH; }

        if (stateManager_.isDataStale(now))
        { systemStateMsg.error_code |= gem_state_msgs::SystemStateStamped::ERROR_STALE_DATA; }

        switch (stateManager_.getSystemState())
        {
            case SystemState::IDLE:
                systemStateMsg.state = gem_state_msgs::SystemStateStamped::STATE_IDLE;
                break;
            case SystemState::RUNNING:
                systemStateMsg.state = gem_state_msgs::SystemStateStamped::STATE_RUNNING;
                break;
            case SystemState::ERROR:
                systemStateMsg.state = gem_state_msgs::SystemStateStamped::STATE_ERROR;
                break;
        }

        systemStatePub_.publish(systemStateMsg);
    }

    void run()
    {
        ros::Rate loopHz(20.0);
        while (ros::ok())
        {
            ros::spinOnce();
            const Time updateTime = fromRosTime(ros::Time::now());
            stateManager_.update(updateTime);
            publishSystemState(updateTime);
            loopHz.sleep();
        }
    }
private:
    ros::NodeHandle nh_;
    StateManager stateManager_;

    ros::Subscriber batteryLevelSub_;
    ros::Subscriber estopSub_;
    ros::Subscriber gpsAccuracySub_;
    ros::Subscriber signalStrengthSub_;
    ros::Subscriber temperatureSub_;

    ros::Publisher systemStatePub_;
};  // class StateManagerNode
}  // class polaris

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_manager_node");

    // Create the state manager node and run
    polaris::StateManagerNode stateManagerNode;
    stateManagerNode.run();

    return 0;
}
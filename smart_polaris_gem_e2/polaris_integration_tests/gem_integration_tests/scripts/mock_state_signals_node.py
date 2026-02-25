#!/usr/bin/env python3
"""
Mock publishers for /battery_level, /gps_accuracy, /signal_strength, /estop, /temperature
for integration tests. All signals are reconfigurable via dynamic_reconfigure so tests
and operators can drive system into IDLE, ERROR and RUNNING modes.
"""
import rospy
from std_msgs.msg import Header
from gem_state_msgs.msg import (
    BatteryLevelStamped,
    GpsAccuracyStamped,
    SignalStrengthStamped,
    EstopStamped,
    TemperatureStamped,
)
from dynamic_reconfigure.server import Server
from gem_integration_tests.cfg import MockStateSignalsConfig


class MockStateSignalsNode:
    def __init__(self):
        self._estop_enabled = False
        self._battery_level = 100
        self._gps_accuracy = 0.0
        self._signal_strength = SignalStrengthStamped.CONNECTED
        self._temperature = 30.0

        self._battery_pub = rospy.Publisher("/battery_level", BatteryLevelStamped, queue_size=1)
        self._gps_pub = rospy.Publisher("/gps_accuracy", GpsAccuracyStamped, queue_size=1)
        self._signal_pub = rospy.Publisher("/signal_strength", SignalStrengthStamped, queue_size=1)
        self._estop_pub = rospy.Publisher("/estop", EstopStamped, queue_size=1)
        self._temperature_pub = rospy.Publisher("/temperature", TemperatureStamped, queue_size=1)

        self._srv = Server(MockStateSignalsConfig, self._reconfigure_cb)
        self._rate = rospy.Rate(20)

    def _reconfigure_cb(self, config, level):
        self._estop_enabled = config.estop_enabled
        # convert from percentage to float between 0.0 and 1.0
        self._battery_level = config.battery_level
        self._gps_accuracy = config.gps_accuracy
        self._signal_strength = config.signal_strength
        self._temperature = config.temperature
        return config

    def _publish(self):
        stamp = rospy.Time.now()
        header = Header(stamp=stamp, frame_id="")

        battery = BatteryLevelStamped(header=header, battery_level=float(self._battery_level) / 100.0)
        gps = GpsAccuracyStamped(header=header, accuracy=self._gps_accuracy)
        signal = SignalStrengthStamped(header=header, strength=self._signal_strength)
        estop = EstopStamped(header=header, enabled=self._estop_enabled)
        temperature = TemperatureStamped(header=header, temperature=self._temperature)

        self._battery_pub.publish(battery)
        self._gps_pub.publish(gps)
        self._signal_pub.publish(signal)
        self._estop_pub.publish(estop)
        self._temperature_pub.publish(temperature)

    def run(self):
        while not rospy.is_shutdown():
            self._publish()
            self._rate.sleep()


def main():
    rospy.init_node("mock_state_signals_node", anonymous=False)
    node = MockStateSignalsNode()
    node.run()


if __name__ == "__main__":
    main()

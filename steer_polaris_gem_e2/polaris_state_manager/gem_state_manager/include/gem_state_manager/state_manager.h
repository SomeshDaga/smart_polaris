#pragma once

#include <gem_state_manager/state_machine.h>
#include <gem_state_manager/time.h>

#include <chrono>
#include <vector>

namespace polaris
{

namespace Thresholds
{
    static constexpr float BATTERY_LOW = 0.5f;
    static constexpr float TEMPERATURE_HIGH = 55.f;
    static constexpr float GPS_BAD_ACCURACY = 200.f;
}

enum class SystemState {
    IDLE, RUNNING, ERROR
};

enum class SignalStrength {
    NOT_CONNECTED, CONNECTED, LOW
};

template <typename T>
struct TimedData
{
    // The rxTime helps to track staleness of data
    Time rxTime;
    T data;

    bool isStale(const Time& now, uint64_t timeoutMs) const
    {
        return (now - rxTime) * 1000. > static_cast<double>(timeoutMs);
    }
};

class StateManager
{
using SMStateT = SystemState;
using SMState = State<SMStateT>;
using SMStatePtr = StatePtr<SMStateT>;

public:
    StateManager(const Time& initTime, uint64_t stateDataTimeoutMs = 0);

    // Update functions
    void updateBatteryLevel(const Time& msgTime, float batteryLevel);
    void updateTemperature(const Time& msgTime, float temperature);
    void updateGpsAccuracy(const Time& msgTime, float accuracy);
    void updateSignalStrength(const Time& msgTime, SignalStrength signal);
    void updateEmergencyStop(const Time& msgTime, bool enabled);

    // Query helpers
    [[nodiscard]] bool isBatteryLow() const;
    [[nodiscard]] bool isTemperatureHigh() const;
    [[nodiscard]] bool isGpsBad() const;
    [[nodiscard]] bool isGpsBadPersistent(const Time& now) const;
    [[nodiscard]] bool isSignalLowPersistent(const Time& now) const;
    [[nodiscard]] bool isSignalNotConnectedPersistent(const Time& now) const;
    [[nodiscard]] bool isEstopEnabled() const;
    [[nodiscard]] bool isDataStale(const Time& now) const;
    [[nodiscard]] bool isTaskRunning() const;

    // Aggregated queries
    [[nodiscard]] bool hasError(const Time& now) const;

    [[nodiscard]] SystemState getSystemState() const;

    void update(const Time& now);

protected:
    StateMachine<SystemState> sm_;
    std::vector<SMStatePtr> states_;

    // Staleness timeout (disabled if set to 0)
    const uint64_t staleDataTimeoutMs_;

    // Sensor data
    TimedData<float> batteryLevel_ = {Time(), Thresholds::BATTERY_LOW};
    TimedData<float> temperature_ = {Time(), Thresholds::TEMPERATURE_HIGH};

    TimedData<float> gpsAccuracy_ = {Time(), Thresholds::GPS_BAD_ACCURACY};
    Time timeLastGpsGood_;

    TimedData<SignalStrength> signalStrength_ = {Time(), SignalStrength::CONNECTED};
    Time timeLastSignalConnected_;
    Time timeLastSignalLow_;

    TimedData<bool> estop_ = {Time(), false};
};
}
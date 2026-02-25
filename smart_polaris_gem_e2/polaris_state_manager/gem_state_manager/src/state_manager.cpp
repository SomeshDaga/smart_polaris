#include <gem_state_manager/state_manager.h>

using namespace polaris;

StateManager::StateManager(const Time& initTime, uint64_t staleDataTimeoutMs)
    : staleDataTimeoutMs_(staleDataTimeoutMs)
{
    // Define all unique states
    auto idleState = std::make_shared<SMState>(sm_, SystemState::IDLE);
    auto runningState = std::make_shared<SMState>(sm_, SystemState::RUNNING);
    auto errorState = std::make_shared<SMState>(sm_, SystemState::ERROR);
    // Register states with the state machine to manage their lifecycle
    idleState->init();
    runningState->init();
    errorState->init();

    // Define transitions from the IDLE state
    idleState->addTransition(runningState, [this](const Time& now) { return isTaskActive() && !hasError(now); });
    idleState->addTransition(errorState, [this](const Time& now) { return hasError(now); });

    // Define transitions from the RUNNING state
    runningState->addTransition(idleState, [this](const Time& now) { return !isTaskActive() && !hasError(now); });
    runningState->addTransition(errorState, [this](const Time& now) { return hasError(now); });

    // Define transitions from the ERROR state
    errorState->addTransition(idleState, [this](const Time& now) { return !isTaskActive() && !hasError(now); });
    errorState->addTransition(runningState, [this](const Time& now) { return isTaskActive() && !hasError(now); });

    // Initialize state machine to the IDLE state
    sm_.initialize(idleState);

    // Initialize timing variables to the initialization time
    timeLastGpsGood_ = initTime;
    timeLastSignalConnected_ = initTime;
}

// Update helpers
void StateManager::updateBatteryLevel(const Time& msgTime, const float batteryLevel)
{ batteryLevel_ = {msgTime, batteryLevel}; }

void StateManager::updateTemperature(const Time& msgTime, const float temperature)
{ temperature_ = {msgTime, temperature}; }

void StateManager::updateGpsAccuracy(const Time& msgTime, const float accuracy)
{
    gpsAccuracy_ = {msgTime, accuracy};
    if (!isGpsBad())
    { timeLastGpsGood_ = msgTime; }
}

void StateManager::updateSignalStrength(const Time& msgTime, const SignalStrength signal)
{
    signalStrength_ = {msgTime, signal};

    if (signal == SignalStrength::CONNECTED)
    { timeLastSignalConnected_ = msgTime; }
}

void StateManager::updateEmergencyStop(const Time& msgTime, const bool enabled)
{ estop_ = {msgTime, enabled}; }

void StateManager::updateHasTask(const Time& msgTime, bool hasTask)
{ hasTask_ = {msgTime, hasTask}; }

// Query helpers
bool StateManager::isBatteryLow() const
{ return batteryLevel_.data < Thresholds::BATTERY_LOW; }

bool StateManager::isTemperatureHigh() const
{ return temperature_.data > Thresholds::TEMPERATURE_HIGH; }

bool StateManager::isGpsBad() const
{ return gpsAccuracy_.data > Thresholds::GPS_BAD_ACCURACY; }

bool StateManager::isGpsBadPersistent(const Time& now) const
{
    return isGpsBad() && (now - timeLastGpsGood_) > 15.0;
}

bool StateManager::isSignalLowPersistent(const Time& now) const
{
    // the signal is considered persistently low if the signal is low/disconnected
    // for > 20 seconds
    return signalStrength_.data == SignalStrength::LOW &&
        now - timeLastSignalConnected_ > 20.0;
}

bool StateManager::isSignalNotConnectedPersistent(const Time& now) const
{
    // the signal is considered persistently lost if the signal is lost
    // for > 10 seconds
    return signalStrength_.data == SignalStrength::NOT_CONNECTED &&
        now - timeLastSignalConnected_ > 10.0;
}

bool StateManager::isEstopEnabled() const
{ return estop_.data; }

bool StateManager::isTaskActive() const
{ return hasTask_.data; }

bool StateManager::isDataStale(const Time& now) const
{
    if (staleDataTimeoutMs_ == 0) { return false; }

    return
        batteryLevel_.isStale(now, staleDataTimeoutMs_) ||
        temperature_.isStale(now, staleDataTimeoutMs_) ||
        gpsAccuracy_.isStale(now, staleDataTimeoutMs_) ||
        signalStrength_.isStale(now, staleDataTimeoutMs_) ||
        estop_.isStale(now, staleDataTimeoutMs_);
}

bool StateManager::hasError(const Time& now) const
{
    return
        isDataStale(now) ||
        isBatteryLow() ||
        isTemperatureHigh() ||
        isGpsBadPersistent(now) ||
        isSignalLowPersistent(now) ||
        isSignalNotConnectedPersistent(now) ||
        isEstopEnabled();
}

SystemState StateManager::getSystemState() const
{ return sm_.getCurrentState(); }

void StateManager::update(const Time& now)
{
    // Update the state machine based on the signal updates
    sm_.update(now);
}
# Polaris State Management and Navigation Planner

This repository builds on the [Polaris GEM e2](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2) Gazebo-based simulator to introduce state management, a high-level task planner and control interfaces. The goal is to build a mobile navigation system that is capable of responding to failure scenarios with a clear separation of responsibilities between software components.

Additionally, rostests and integration tests are introduced to evaluate the impact of the components introduced in this repository.

> **Note**: This project has been tested inside an ARM64 Ubuntu 24.04 Virtual Machine running on VMWare Fusion 13

## Project Setup

For first time setup, please clone this repository and resolve submodules using:

```bash
git clone --recursive https://github.com/SomeshDaga/smart_polaris.git
```

The provided Dockerfile provides support for development and production workflows.

```bash
# For development builds
docker build --build-arg BUILD_DEV_IMAGE=true -t smart_polaris:dev .
# For production builds
docker build -t smart_polaris:prod .
```

Convenience scripts provided in `runners/` can be used for building dev and prod images, and running targeted ROS/Gazebo workloads inside these containers.

### ROS and Gazebo

This project uses the Noetic ROS 1 distribution with Gazebo 11 (as the simulator had been developed and tested for these versions). It is recognized that the ROS distro has reached End-of-Life (EOL), as mainstream ROS development has largely transitioned to ROS 2.

With this in mind, software developed in this repository aims to keep components ROS-agnostic as much as possible, with thin ROS 1 wrappers for node execution. This will be described in more detail below.

### Project Organization

The high-level structure of the repository is as follows:

```
.
├── Dockerfile               # Build dev and production docker images with full simulator + software
├── entrypoint.sh            # Docker entrypoint script
├── polaris_gem_e2           # Git submodule of the original polaris_gem_e2 ROS Workspace
├── README.md                # Detailed project description and instructions
├── resources                # Resources for managing docker builds e.g. rosdep key deps
├── runners                  # Convenience scripts to run ROS and Gazebo workloads inside docker
└── smart_polaris_gem_e2     # ROS Workspace containing new features for the Polaris Gem e2
```

### Docker Builds

As previously mentioned, two types of docker image building is supported for this project: Development and Production.

Additionally, the base docker image was carefully chosen, with upstream docker repositories supporting both ARM64 and AMD64 architectures for the image. Hence, the Dockerfile should work out-of-the-box on both system architectures.

#### Development Images

Features of the development images:

* Non-root user with same `user` and `group` id as host system (allows bind-mounted files and folders to be used without permission conflicts both inside docker and on the host system)
* Non-root user has passwordless sudo access inside container to install system packages as needed
* Debug-mode builds of ROS workspaces

#### Production Images

Production images use non-root users without sudo access:

* Cannot install system packages inside the docker container (less attack surface for vulnerabilities)
* Release-mode optimized builds of ROS workspaces

### Runner Scripts

A few convenience scripts are provided inside the `runners/` directory to execute specific ROS/Gazebo workloads inside a docker instance, with X11 forwarding (for GUI-based applications) to the host system where applicable:

1. `launch_integration_run.sh`: Launches system with State Manager, High-level Task Planner, Path-Tracking controller and GUI-based simulation with Gazebo, RViz and RQT. Users can interactively engage with the system to induce failure scenarios (using RQT window) and observe feedback on waypoint navigation and state management (via Gazebo and custom RViz visualization).
2. `run_state_manager_rostests.sh`: Run ROS-based unit tests for State Manager with explicit failure scenarios
3. `run_integration_tests.sh`: Runs integration tests with State Manager, High-level Task Planner, Path Tracking Controller and a headless Gazebo server.

### ROS Workspaces

> **Note**: The [Polaris GEM e2](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2) is available for public use. However, the respository resides under an organization gitlab account for the University of Illinois at Urbana-Champaign. Forking for the respository is disabled, which motivated the use of the setup described below.


As the [Polaris GEM e2](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2) cannot be forked and achieving a similar result would rely on manual mirroring of the repository to a different upstream target, the following approach was favoured instead:

1. The [Polaris GEM e2](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2) is pulled into this repository as a git submodule (named `polaris_gem_e2` in the root folder) 
2. The `polaris_gem_e2` folder represents an independent `catkin` workspace that is not modified
3. Software developed as an extension to the original project is contained in a separate workspace inside the `smart_polaris_gem_e2` folder
4. Both folders are copied into the docker image during build time, with the `smart_polaris_gem_e2` workspace extending the `polaris_gem_e2` workspace to allow for interoperability and dependency management

## Smart Polaris Modules

To date, the following modules have been added under the `smart_polaris_gem_e2` folder to extend the capabilities of the original Polaris GEM e2 project and are documented below along with rationale behind design decisions.

In general, the architecture was motivated such that the system (to the greatest degree) is:

1. **Observable**: Input/Output signals and System Behaviour can be visualized and recorded
2. **Measurable**: Timing of signals and propagation through the system impact system behaviour can be profiled/audited
3. **ROS Agnostic**: To simplify migration to ROS 2 (at a later time), use of ROS-based constructs (e.g. publishing, subscribing, callbacks etc.) should be decoupled from system logic as much as possible
4. **Testable**: To enable translation of this project to real mobile platforms for a variety of use cases, the system needs to be testable. A number of realisitic failure scenarios are generated and tested via structured tests for these new modules


### GEM State Manager

The GEM State Manager, provided by the `gem_state_manager` package provides a State Management system that can react to failure scenarios through the consumption of signals represented by ROS topics. At a high-level, it determines the state of the system to be in one of 3 states based on input signals:

1. `IDLE`: No failure condition -AND- No navigation task running
2. `RUNNING`: No failure condition -AND- Navigation task running
3. `ERROR`: Failure condition exists

#### Running the State Manager and Tests

The `state_manager_node` can be run via:

```
rosrun gem_state_manager state_manager_node
```

Rostests for the `state_manager_node` can be run using:

```
catkin test -i gem_state_manager
```

Alternatively, the `runners/run_state_manager_rostests.sh` convenience script can be used.

> **Note**: Due to time-based failure scenarios, the tests may take up to 5 mins to run

#### Input/Output Messages and Failure Scenarios

The following failure test scenarios were created for this system, along with timestamped (for measurability) message definitions (defined in `gem_state_msgs` package):

1. Battery Level (**Topic**: /battery_level): `ERROR` when `battery_level < 50%`
    ```
    ## BatteryLevelStamped.msg ##
    std_msgs/Header header
    # Battery level specified between 0.0-1.0 (0% to 100%)
    float32 battery_level
    ```
2. Emergency Stop (**Topic**: /estop): `ERROR` when `enabled = true`
    ```
    ## EstopStamped.msg ##
    std_msgs/Header header
    # True if estop is enabled, false otherwise
    bool enabled
    ```
3. Gps Accuracy (**Topic**: /gps_accuracy): `ERROR` when `accuracy > 200mm (for >= 15s)`
    ```
    ## GpsAccuracyStamped.msg ##
    std_msgs/Header header
    # Positive floating point number representing gps accuracy in mm
    float32 accuracy
    ```
4. Signal Strength (**Topic**: /signal_strength):
    - `ERROR` when `strength = LOW_SIGNAL (for >= 20s)`
    - `ERROR` when `strength = NOT_CONNECTED (for >= 10s)`
    ```
    ## SignalStrengthStamped.msg ##
    # Constants defining signal strength
    uint8 NOT_CONNECTED=0
    uint8 CONNECTED=1
    uint8 LOW_SIGNAL=2

    std_msgs/Header header
    # Signal strength is represented by one of the constants defined above
    uint8 strength
    ```
5. Temperature (**Topic**: /temperature): `ERROR` when `temperature > 55°C`
    ```
    ## TemperatureStamped.msg ##
    std_msgs/Header header
    # Temperature in deg C
    float32 temperature
    ```

Additionally, the state manager subscribes to a status message (**Topic**: /task_planner_status) from the Navigation Task Planner (described later):

```
## TaskPlannerStatus.msg ##
std_msgs/Header header

uint8 STATUS_NO_TASK=0
uint8 STATUS_PAUSED_TASK=1
uint8 STATUS_RUNNING_TASK=2

uint8 status
```

The state manager node publishes (**Topic**: /system_state) the following message describing the state of the system:

```
## SystemStateStamped.msg ##
std_msgs/Header header

# The following error codes represent bit masks
# We can encode one or more of these in the error_code field
uint8 ERROR_NONE=0
uint8 ERROR_BATTERY_LOW=1
uint8 ERROR_ESTOP=2
uint8 ERROR_GPS_LOST=4
uint8 ERROR_SIGNAL_LOST=8
uint8 ERROR_SIGNAL_LOW=16
uint8 ERROR_TEMPERATURE_HIGH=32
# This is a generic stale data error if any of the input streams
# become stale
uint8 ERROR_STALE_DATA=64

# The system can be in one of the following states
uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_ERROR=2

uint8 error_code
uint8 state
```

> **Note**: In addition to the high level `state` (which can be `STATE_IDLE`, `STATE_RUNNING` or `STATE_ERROR`), we can also encode zero, one or more erors in the `error_code` field. We define each error as a bit in a bit mask, which is represented by the `error_code` field. This helps to improve observability of the system.


#### Software Design

The package structure of `gem_state_manager` is shown below:

```
gem_state_manager
├── CMakeLists.txt
├── include
│   └── gem_state_manager
│       ├── ros_time_helpers.h          # Converter between ros::Time and polaris::Time
│       ├── state_machine.h             # Templated ROS-agnostic State Machine implementation
│       ├── state_manager.h             # Header for ROS-agnostic state manager
│       └── time.h                      # C++ Chrono-based polaris::Time class
├── nodes
│   └── state_manager_node.cpp          # ROS Node for State Manager (uses state_manager.h)
├── package.xml
├── src
│   └── state_manager.cpp               # Implementation for ROS-agnostic state manager
└── test
    ├── state_manager_scenarios.cpp     # ROS-based unit tests for state manager
    └── state_manager_scenarios.test    # Tests launch for unit tests alongside state manager

```

##### Chrono-based Time Class

In addition to recognizing failure scenarios from observing error signals (described above), for safety, it is also critical to understand if certain messages are not being received in a timely manner i.e. sensor data is stale.

To detect staleness, the ROS-agnostic state manager implementation should be able to work without ros::Time. Hence, a custom std::chrono based `polaris::Time` class was introduced (with operator overloading to compare and subtract time objects).

```cpp
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

    ...

```

In addition, helpers to convert between `ros::Time` and `polaris::Time` were introduced in `ros_time_helpers.h` i.e.

```cpp
/**
* @brief Helper function to convert from ros::Time to polaris::Time
*/
static Time fromRosTime(const ros::Time& t)
{
    return Time(Time::TimePoint{} +
            std::chrono::seconds(t.sec) +
            std::chrono::nanoseconds(t.nsec));
}
```

This will help to simplify migration to ROS 2, through introducing similar helpers to convert between native ROS 2 Time objects and std::chrono.

##### State Machine

A generic State Machine module was implemented using the 'State' behavioural pattern for states to created transitions inside a state machine based on user-defined events. A conceptual overview of the implementation is given below:

```cpp
namespace polaris
{

class Time;

// Event callback evaluated during update()
using Event = std::function<bool(const Time&)>;

template<typename StateT>
class StateMachine;

template<typename StateT>
class State : public std::enable_shared_from_this<State<StateT>>
{
public:
    State(StateMachine<StateT>& sm, StateT state);

    /// Register state with the state machine
    void init();

    /// Add transition to another state
    void addTransition(
        std::shared_ptr<State<StateT>> toState,
        Event eventFn);

    /// Evaluate events and trigger transitions
    void processEvents(const Time& now);

    /// Retrieve underlying state value
    StateT getState() const;
};


template<typename StateT>
class StateMachine
{
public:
    StateMachine();

    /// Set initial active state
    void initialize(std::shared_ptr<State<StateT>> initialState);

    /// Process events for current state
    void update(const Time& now);

    /// Get current active state
    StateT getCurrentState() const;

private:
    friend class State<StateT>;

    /// Change active state of state machine (usually called by a State)
    void setCurrentState(StatePtr<StateT> newState)
};

} // namespace polaris
```

As a quick overview of this design:

1. An `Event` is represented by a user-defined function (allowing flexibility in definition), that takes in a `Time` and returns a `bool`. A `true` return value indicates that the event has fired.
2. A `State` defines transitions to other states, with a corresponding `Event` defined for each transition. Each `State` object also holds a reference to the `StateMachine` they are a part of. Invoking `processEvents` allow a `State` to process its events and transition the `StateMachine` to another state as required.
3. The `StateMachine` is a high-level object that manages the lifetime of all states, and holds a reference to the current active state. On each `update()`, only the events associated with the active state have to be evaluated.

##### State Manager

The State Manager implements a application specific but ROS-agnostic state management system in pure C++. It provides interfaces using standard C++ functions and types to update signal states (for battery level, estop etc.). It uses the State Machine implementation (described above) with the templated type resolved to:

```cpp
// Possible states that the state manager can transition between
enum class SystemState {
    IDLE, RUNNING, ERROR
};
```

The use of the State Machine is as follows:

```cpp
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
```

The state manager provides C++ setters to update its knowledge of different signals received by the state_manager_node e.g.

```cpp
// Update functions
void updateBatteryLevel(const Time& msgTime, float batteryLevel);
void updateTemperature(const Time& msgTime, float temperature);
void updateGpsAccuracy(const Time& msgTime, float accuracy);
void updateSignalStrength(const Time& msgTime, SignalStrength signal);
void updateEmergencyStop(const Time& msgTime, bool enabled);
void updateHasTask(const Time& msgTime, bool hasTask);

// Query helpers
[[nodiscard]] bool isBatteryLow() const;
[[nodiscard]] bool isTemperatureHigh() const;
[[nodiscard]] bool isGpsBad() const;
[[nodiscard]] bool isGpsBadPersistent(const Time& now) const;
[[nodiscard]] bool isSignalLowPersistent(const Time& now) const;
[[nodiscard]] bool isSignalNotConnectedPersistent(const Time& now) const;
[[nodiscard]] bool isEstopEnabled() const;
[[nodiscard]] bool isDataStale(const Time& now) const;
[[nodiscard]] bool isTaskActive() const;
```

##### State Manager Node

The state manager node represents the ROS "glue" that subscribes to topics and updates the state of the state machine with the provided setters.

The `/system_state` topic is published at a fixed frequency of 20Hz. A fixed frequency is used in favour of ad-hoc message publication as it supports measurability of the node loop rate and improves diagnostic capability of the system.

Refer to `state_manager_node.cpp` in `gem_state_manager` for details on the implementation.


### Navigation Task Planner and Controller Interface

The original [Polaris GEM e2](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2) project provides path-tracking controllers that are able to use a series of poses (or waypoints) to perform navigation. More specifically, ROS-enabled implementations of the `Pure Pursuit` and `Stanley` controllers are provided.

No high-level navigation planning is provided.

To respond to transitions in the system state (as published by the state manager), a navigation task planner was required to provide high-level task planning by issuing or preempting goals to low-level path-tracking controllers.

#### Interface between Task Planner and Path-Tracking Controllers

To decouple planning and controllers, it was important to define an interface between them.

Since waypoint navigation is a long horizon task, a ROS action interface best fit this use case.

A `gem_controller_action` package was introduced with the following action definition:

```
## NavigateWaypoints.action ##

# Goal
# Series of waypoints to navigate
geometry_msgs/Pose2D[] waypoints

# Index of waypoint to start
uint64 start_index
---
# Result
# true = goal completed
# false = goal aborted by controller
bool success
---
# Feedback
# Current tracked waypoint index
uint64 target_index

# Percentage of waypoints navigated
float32 progress
```

Since path-tracking controllers like Pure Pursuit and Stanley typically require access to a path (rather than a single waypoint), the goal encompasses a list of waypoint poses.

Additionally, since a goal may be preempted (if it is unsafe to navigate), a `start_index` is provided to resume the waypoint navigation. The previous goal's feedback provides a `target_index` representing the progress achieved to be served as the `start_index` for the next run if a navigation task is resumed.

The path-tracking controllers host the ROS action server `/navigate_waypoints`. An ROS action client hosted in the Task Planner serves and preempts goals depending on the system state. 

#### Task Planner

A `gem_task_planner` package was introduced with a rospy-based `task_planner_node`.

It can be run using:

```bash
rosrun gem_task_planner task_planner_node.py
```

The node performs the following functions:

- Subscribes to `/system_state`
- Publishes to `/task_planner_status` (fixed frequency of 20Hz) indicating its status i.e. `STATUS_NO_TASK`, `STATUS_PAUSED_TASK` or `STATUS_RUNNING_TASK`
- Advertises service clients:
    - `/serve_waypoints`: To initiate new tasks (or replace ongoing task)
        ```
        ## ServeWaypoints.srv ##
        # csv file w/ x,y,yaw poses
        string waypoints_file
        ---
        bool success
        ```
    - `/cancel_tasks` (uses `std_srvs/Empty`): To allow user-generated task cancellation
- Hosts action client to `/navigate_waypoints` to send and preempt goals to path-tracking controller(s)
- Subscribes to `/navigate_waypoints/feedback` and `/navigate_waypoints/result` to cache goal feedback and process preemption, completion or abortion of goals.

The behaviour of the task planner for /system_state errors and recovery are as follows:

1. Error State Triggers
    - Active controller goals are preempted (goes to status: `STATUS_PAUSED_TASK`)
    - Progress of preempted goals are cached
2. Error State Recovery
    - Preempted goals are resumed from cached progress (goes to status: `STATUS_RUNNING_TASK`)
    - New goals are started immediately

#### ROS-Agnostic Path Tracking Controllers

To enable ROS-agnostic path controller implementations (i.e. decouple ROS functionality from controller logic), an abstract rospy interface was introduced, `controller_action_interface.py` in the `gem_controller_action` package.

This abstract python class performs the following functions:
- Action server lifecycle
- Action Goal execution loop (handles preemptions/completion)
- Robot state retrieval (from `/gazebo/get_model_state`)
- Command publishing (to `/gem/ackermann_cmd`)
- Feedback reporting (to `/navigate_waypoints/feedback`)

Controller implementations only have to derive from this ROS-enabled abstract class and implement the ROS-agnostic function (with class member access to the waypoints):

```python
@abstractmethod
def control_step(self,
    state: RobotState,
    start_idx: int) -> ControllerState:
```

THe control frequency of the loop is fixed and managed by the abstract base class (in the goal execution loop).

ROS-enabled python controller implementations from `gem_pure_pursuit_sim` and `gem_stanley_sim` were migrated into new packages `gem_pure_pursuit_sim_v2` and `gem_stanley_sim_v2`, with implementation trimmed down to implement the `control_step(...)` method and derive from the `ControllerActionInterface` class.

By reusing a common ROS interface across the controllers, we can switch between both controllers flexibly between separate runs.

Path-tracking controllers w/ action servers can be launched using:

```bash
# Pure pursuit controller
rosrun gem_pure_pursuit_sim_v2 pure_pursuit_sim.py
# Stanley controller
rosrun gem_stanley_sim_v2 stanley_sim.py
```

### System Integration

#### Running

To run the system end-to-end, both for interactive simulation and integration tests, the `gem_integration_tests` package was introduced.

> Note: X11 forwarding is required to run GUI-enabled integration runs with interactivity.

For convenience,
- Interactive simulation with X11 forwarding can be launched with `runners/launch_integration_run.sh`
- Integration tests can be launched with `runners/run_integration_tests.sh`

Results from integration tests are included in the [Performance Report](https://docs.google.com/document/d/1WDdFwSpNwf5ZT2EGFDyTaDiJ88Gau3d0w6FHBD62BJQ/edit?usp=drivesdk).

#### Visualization

To support visualization of waypoint navigation goal/progress, task planner status, error states and mock signals, a `status_overlay_node.py` was introduced in the `gem_integration_tests` package. This published an `jsk_rviz_plugins/OverlayText` message with aggregated system information.

An RViz file was created in `rviz/gem_integration_tests.rviz` to load the required plugins and topics on launch.

Additionally, to enable interactivity for the manual evaluation of failure test scenarios, a `mock_signals_state_node.py` was introduced to publish mock signal input for the state manager. The signal values are configurable via `dynamic_reconfigure`.

This also allowed for the use of the `rqt_reconfigure` plugin with `rqt` to interactively change signal values and observe impact on the system.

## Performance Profiling

For performance profiling methodology and results, please refer to [Performance Report](https://docs.google.com/document/d/1WDdFwSpNwf5ZT2EGFDyTaDiJ88Gau3d0w6FHBD62BJQ/edit?usp=drivesdk).

> Future Improvement: Scenarios tests from `gem_integration_tests` can be reused for generating data for measuring profiling performance.

## Challenges Encountered

### ARM64-based development on Virtual Machine

During development, I did not have access to a Linux machine. Instead, I used a Macbook M1 Pro that uses an ARM64 architecture.

ROS 1 support is non-existent for non-linux machines. Therefore, I decided to use VMWare Fusion on my Macbook to run an Ubuntu VM.

Ubuntu Desktop ISOs for ARM64 prior to Ubuntu 24.04 are also not readily available in official repositories (mainly server ISOs exist). My initial goal was to work with a Ubuntu 20.04 system to allow for "local" development, and later packaged via docker.

In the end, I got around this challenge by developing a Dockerfile that is applicable to both development and production goals. With the development images, I was able to bind mount ROS workspaces from the host VM system into the containers.

### ARM64 ROS 1 Docker Images

Additionally, I found that `osrf` maintains the ROS 1 full desktop images (with Gazebo and other gui tools included), but only for AMD64 distributions.

There were unofficial packages available for ARM64 platforms [seunmul/ros:noetic-full-arm64](https://hub.docker.com/layers/seunmul/ros/noetic-full-arm64/images/sha256-506e41fbe11809e0d1a89d49284fdf4904f18f8079bdf13f11424eca96c761a4?context=explore).

However, I wanted the Dockerfile to be reusable across all system architectures and opted to use official docker images from `ros` with less vulnerabilities and amd64 + arm64 support. I opted to install additional dependencies manually inside the Dockerfile e.g. for Gazebo.

### Profiling Bugs and Iteration

For profiling system performance across a number of failure scenarios, I wanted to run each failure scenario for each signal a large number of times; in order to generate a representative statistical distribution of system response times for analysis.

This is a time consuming process, especially with prolonged time-based failure conditions for `/gps_accuracy` and `/signal_strength`.

Initially, I approached it by aggregating metrics during the simulation of failure scenarios. However, this led to a loss of raw data and inflexibility to tweak analysis of the data, thereby needing to repeat experimental runs.

I switched to the better approach of performing rosbag records of each simulated scenario and performing the analysis offline. Even with this approach, I ran into issues with `rosbag record` missing messages (extent depending on order of topic subscription) during its startup. This was identified by inspecting rosbag files with the `rqt_bag` plugin. Hence, this approach also offered between debugging and introspection capabilities.
#pragma once

#include <gem_state_manager/time.h>

#include <functional>
#include <map>
#include <memory>

namespace polaris
{

template<typename StateT>
class State;

template <typename StateT>
using StatePtr = std::shared_ptr<State<StateT>>;

template <typename StateT>
class StateMachine;

using Event = std::function<bool(const Time&)>;

template <typename StateT>
class State : public std::enable_shared_from_this<State<StateT>>
{
public:
    State(StateMachine<StateT>& sm, StateT state)
        : sm_(sm),
          state_{state}
    {}

    // Add a transition to another state based on an event
    void addTransition(StatePtr<StateT> toState, Event eventFn)
    {
        stateToEventMap_.emplace(toState, eventFn);
    }

    // Processes all registered events to transition state
    // machine to a different state if needed
    void processEvents(const Time& now)
    {
        for (auto& stateEventP : stateToEventMap_)
        {
            auto& toState = stateEventP.first;
            auto& eventFn = stateEventP.second;
            if (eventFn(now))
            {
                sm_.setCurrentState(toState);
                break;
            }
        }
    }

    StateT getState() const { return state_; }

private:
    StateMachine<StateT>& sm_;
    const StateT state_;
    std::map<StatePtr<StateT>, Event> stateToEventMap_;
};

template <typename StateT>
class StateMachine
{
public:
    StateMachine() {}

    void initialize(StatePtr<StateT> initialState)
    { currentState_ = initialState; }

    // Process events for the current active state
    // and transition to another state if needed
    void update(const Time& now)
    {
        currentState_->processEvents(now);
    }

    [[nodiscard]] StateT getCurrentState() const
    { return currentState_->getState(); }

private:
    friend class State<StateT>;

    void setCurrentState(StatePtr<StateT> newState)
    { currentState_ = newState; }

    StatePtr<StateT> currentState_ = nullptr;
};
}  // namespace polaris
/**
 * @file state_machine.h
 * @brief Provides a templated State and StateMachine implementation
 *
 * Provides a StateMachine that utilizes the 'State' Behaviour Pattern
 * where Events registered against States can change the active
 * state of a State Machine.
 *
 * Events are defined as function callbacks that can be flexibly defined
 * during the creation and configuration of each State
 *
 * Templated classes for State and StateMachine allow flexibility in creating
 * state machines for user-defined types
 */
#pragma once

#include <gem_state_manager/time.h>

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

namespace polaris
{
template<typename StateT>
class State;

// Alias for shared_ptr to State<StateT>
template <typename StateT>
using StatePtr = std::shared_ptr<State<StateT>>;

// Forward definition for StateMachine<StateT>
template <typename StateT>
class StateMachine;

// Events are defined as function callbacks that take accept
// a time input and output a boolean
// Return value of true indicates firing of the event
using Event = std::function<bool(const Time&)>;

// States are meant to be created using shared_ptr objects
// This is to enable reference counting to ensure States stay alive
// for the lifetime of the StateMachine against which they are registered
template <typename StateT>
class State : public std::enable_shared_from_this<State<StateT>>
{
public:
    State(StateMachine<StateT>& sm, StateT state)
        : sm_(sm),
          state_{state}
    {}

    /**
    * @brief Register this state with the state machine
    *
    * Register the state against the state machine is mandatory to
    * ensure the state machine handles the lifetime of the state
    */
    void init()
    {
        sm_.addState(this->shared_from_this());
    }

    /**
    * @brief Register a transition to a state w/ corresponding event
    *
    * Events registered via this function are evaluated in processEvents(...)
    * to check for state transitions
    *
    * @param toState State to move to if event fires
    * @param eventFn Function representing event. Event fires if function evaluates to true
    * @throw std::logic_error if the state has not been registered against the state machine
    */ 
    void addTransition(StatePtr<StateT> toState, Event eventFn)
    {
        if (!sm_.containsState(this->shared_from_this()))
        {
            throw std::logic_error("addTransition: State not registered with state machine");
        }
        stateToEventMap_.emplace(toState, eventFn);
    }

    /**
    * @brief Evaluates registered events and transitions state machine to state corresponding
    *        to first firing event
    *
    * @param now Current time as input to event callbacks
    */ 
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
    // State machine holding this state
    StateMachine<StateT>& sm_;
    // The underlying user-defined state value held by this state
    const StateT state_;
    // Map of states that can be transitioned to and their corresponding
    // events that need to fire to initiate the transition
    std::map<StatePtr<StateT>, Event> stateToEventMap_;
};

// A Finite State Machine implementation for user-defined state types
template <typename StateT>
class StateMachine
{
public:
    StateMachine() {}

    /**
    * @brief Set initial state of the state machine
    *
    * @param initialState Initial state for the state machine
    */ 
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
    // Allow state objects to modify the state machine
    // This is used to enable the 'State' behaviour pattern
    friend class State<StateT>;

    /**
    * @brief Adds new state to the state machine
    *
    * The state machine stores all registered states to ensure
    * the ref count of the states never drops to 0 until the
    * state machine is destroyed
    *    
    * @param state New state for the state machine
    * @throw std::logic_error if the state to add has already been registered
    */ 
    void addState(StatePtr<StateT> state)
    {
        if (containsState(state))
        {
            throw std::logic_error("Adding same state twice to state machine!");
        }
        states_.push_back(state);
    }

    /**
    * @brief Set state of the state machine
    *    
    * @param newState New active state for state machine
    */ 
    void setCurrentState(StatePtr<StateT> newState)
    { currentState_ = newState; }

    /**
    * @brief Checks if a state has already been added to the state machine
    *    
    * @param state The state to search for
    */ 
    [[nodiscard]] bool containsState(StatePtr<StateT> state) const
    { return std::find(states_.begin(), states_.end(), state) != states_.end(); }

    StatePtr<StateT> currentState_ = nullptr;
    std::vector<StatePtr<StateT>> states_;
};
}  // namespace polaris

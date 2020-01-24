#ifndef AGENTENVIRONMENT_H
#define AGENTENVIRONMENT_H

#include <vector>

/* 
 * Define types
 */
typedef int action_type;
typedef float reward_type;

class EnvState
{
public:
    reward_type reward;
    bool done;
    bool success;
};

template< typename T >
class TypedEnvState : public EnvState
{
public:
    std::vector<T> state;
};

/*
 * Base class for any Agent Envrionment
 * Describes how Agent observes envrionment and executes actions
 */
class AgentEnvironment {
public:
    AgentEnvironment(int n_actions, int dim_states)
        : n_actions(n_actions), dim_states(dim_states) { }
    virtual ~AgentEnvironment() { }

    /* return dimention of action space */
    int get_n_actions() const { return n_actions; }

    /* return dimension of state space */
    int get_dim_states() const { return dim_states; }

    /* perform one step (apply action) */
    virtual void execute(action_type action) = 0;

    /* returns the new observations */
    virtual bool observe(EnvState& env_state) = 0;

    /* reset environment and return to inital observation */
    virtual void reset() = 0;

private:
    int n_actions;  // number of actions
    int dim_states; // dimension of state vector
};

#endif // AGENTENVIRONMENT_H

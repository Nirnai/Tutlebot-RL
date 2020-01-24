#ifndef DQNLEARNER_H
#define DQNLEARNER_H

#include "turtlebot_dqn/matstruct.h"
#include "turtlebot_dqn/turtlebot_lidar_dqn_env.h"
#include "turtlebot_dqn/dqn.h"
#include "MiniDNN/Network.h"

class DQNLearner
{
public:
    DQNLearner(AgentEnvironment* agentenv, const std::string& log_dir = "", int net_save_steps = 2000);
    ~DQNLearner();

    /* start training session */
    void start(int total_episodes, int max_steps);
    /* main update loop call this function as fast as possible */
    int update(bool user_action = -1);

    /* save all data */
    int save();

private:
    AgentEnvironment* agentenv;
    dqn::DDQN ddqn;

    bool is_started, is_after_reset;
    int total_episodes, max_steps;
    int cnt_episodes, cnt_steps;

    float alpha;
    float gamma;
    float epsilon_discount;

    action_type action;
    TypedEnvState<dqn::scalar_t> env_state;
    std::vector<dqn::scalar_t> state;
    std::vector<dqn::scalar_t> env_inital_state;

    std::string log_dir;
    int net_save_steps;

    /* monitored variables */
    float cumulated_reward;
    std::vector<float> rewards;
    std::vector<float> fitting_errs;
    std::vector<float> epsilons;
    std::vector<int> num_of_steps;
    std::vector<int> selected_actions;
    std::vector< std::vector<float> > q_value_start;
    std::vector< std::vector<float> > q_value_goal;

    MiniDNN::Network* build_network();
};

#endif // DQNLEARNER_H

#ifndef DQN_H
#define DQN_H

#include "MiniDNN/MiniDNN.h"
#include "turtlebot_dqn/replay.h"

namespace dqn {

/*
 * Basic QLearning
 * Takes pointer to network object
 */
class DQN
{
public:
    DQN(int batch_size, float epsilon, float alpha, float gamma );
    DQN(MiniDNN::Network* net, int batch_size, float epsilon, float alpha, float gamma );
    virtual ~DQN();

    void setEpsilon(double epsilon) { this->epsilon = epsilon; }
    double getEpsilon() const { return epsilon; }

    /* update step */
    float learn(const std::vector<scalar_t>& state, const label_t& action, const scalar_t& reward, const std::vector<scalar_t>& next_state, bool isfinished);

    /* action wtth hightes q value */
    int generate_action(const Matrix& state);
    int generate_action(const std::vector<scalar_t>& state);

    /* get learned policy for state */
    int policy(const std::vector<scalar_t>& state);

    /* returns q value of given state action pair (can be used to monitor state action pairs) */
    double getQ(const std::vector<scalar_t>& state, int action);

    int set_net(MiniDNN::Network* net);
    const MiniDNN::Network* get_net() const { return net; }

    int set_replay(Replay* replay);
    const Replay* get_replay() { return replay; }

protected:
    MiniDNN::Network *net;              // Q fnc approximation
    Replay *replay;                     // replay buffer
    MiniDNN::RMSProp opt;               // optimaization

    int action_size;
    int batch_size;
    int state_size;

    int learn_start;
    float epsilon;
    float alpha;
    float gamma;

    std::random_device rd;  //  (seed) engine
    std::mt19937 rng;       // random-number engine
    std::uniform_int_distribution<int> int_uni;
    std::uniform_real_distribution<double> real_uni;

    /* replay sampled history */
    virtual float memory_replay();

private:
    /* return the target values and optimal actions */
    Matrix qtarget(const Matrix& rewards, const Matrix& qValues, const Matrix& next_qValues, const Matrix& actions, const std::vector<uint8_t>& isfinished, float gamma);
};

/*
 * Double QLearning
 */
class DDQN : public DQN
{
public:
    DDQN(MiniDNN::Network* net, MiniDNN::Network* target_net, int batch_size, int swap_cnt, float epsilon, float alpha, float gamma );
    DDQN(int batch_size, int swap_cnt, float epsilon, float alpha, float gamma );
    virtual ~DDQN();

    int set_target_net(MiniDNN::Network* target_net);
    const MiniDNN::Network* get_target_net() const { return target_net; }

protected:
    /* replay sampled history */
    virtual float memory_replay();

private:
   MiniDNN::Network* target_net;    // target network, keep stable till swap_cnt reached
   int swap_cnt;
   int cnt;

   /* return the target values and optimal actions */
   Matrix qtarget(const Matrix& rewards, const Matrix& qValues, const Matrix& next_qValues, const Matrix& next_qValues_t, const Matrix& actions, const std::vector<uint8_t>& isfinished, float gamma);
};

}

#endif // DQN_H

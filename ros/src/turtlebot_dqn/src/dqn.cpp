#include "turtlebot_dqn/dqn.h"
#include <iostream>

using namespace MiniDNN;
using namespace dqn;

DQN::DQN(Network* net, int batch_size, float epsilon, float alpha, float gamma )
    : net(net), batch_size(batch_size), epsilon(epsilon), alpha(alpha), gamma(gamma)
{
    state_size = net->in_size();
    action_size = net->out_size();

    rng = std::mt19937(rd());
    int_uni = std::uniform_int_distribution<int>(0, action_size-1);
    real_uni = std::uniform_real_distribution<double>(0, 1);

    opt.m_lrate = alpha;
    opt.m_decay = 0.99;
}

DQN::DQN(int batch_size, float epsilon, float alpha, float gamma )
    : net(NULL), replay(NULL), batch_size(batch_size), epsilon(epsilon), alpha(alpha), gamma(gamma)
{
    rng = std::mt19937(rd());
    real_uni = std::uniform_real_distribution<double>(0, 1);

    opt.m_lrate = alpha;
    opt.m_decay = 0.99;
}

DQN::~DQN()
{
    delete net;
    delete replay;
}

int DQN::set_net(MiniDNN::Network* net)
{
    this->net = net;

    state_size = net->in_size();
    action_size = net->out_size();
    if(replay)
        replay->initialize( state_size );
    int_uni = std::uniform_int_distribution<int>(0, action_size-1);
    return 0;
}

int DQN::set_replay(Replay* replay)
{
    // only set replay buffer if size if correct
    if(replay->get_state_size() != state_size) {
        return -1;
    }
    this->replay = replay;
    return 0;
}

float DQN::learn(const std::vector<scalar_t>& state, const label_t& action, const scalar_t& reward, const std::vector<scalar_t>& next_state, bool isfinished)
{
    // 1. convert to eigen matrix
    const Vector state_m = Eigen::Map<const Vector, Eigen::Unaligned>(state.data(), state.size());
    const Vector next_state_m = Eigen::Map<const Vector, Eigen::Unaligned>(next_state.data(), next_state.size());

    // 2. remember transition
    replay->push_back(state_m, action, reward, next_state_m, 0.0f, isfinished);
    if(replay->is_filled())
        return memory_replay(); // learning !!!
    return 0.0f;
}

int DQN::policy(const std::vector<scalar_t> &state)
{
    int i, j;
    const Vector state_m = Eigen::Map<const Vector, Eigen::Unaligned>(state.data(), state.size());
    Matrix qValues = net->predict(state_m);
    qValues.maxCoeff( &i, &j );
    return i;
}

double DQN::getQ(const std::vector<scalar_t>& state, int action)
{
    const Vector state_m = Eigen::Map<const Vector, Eigen::Unaligned>(state.data(), state.size());
    Matrix qValues = net->predict(state_m);
    return qValues(action);
}

float DQN::memory_replay()
{
    // 1. get samples form history buffer
    const Memory& samples = replay->sample();

    // 2. evaluate neural net at all states / next states
    Matrix qValues = net->predict( samples.states ); // qValues of state
    Matrix next_qValues = net->predict( samples.next_states ); // qValues of next state

    // 3. compute target values
    Matrix targets = qtarget(samples.rewards, qValues, next_qValues, samples.actions, samples.isfinished, gamma);

    // 4. update network
    net->fit(opt, samples.states, targets, batch_size, 1, int_uni(rng));
    return net->get_output()->loss();
}

Matrix DQN::qtarget(const Matrix& rewards, const Matrix& qValues, const Matrix& next_qValues, const Matrix& actions, const std::vector<uint8_t>& isfinished, float gamma)
{
    Matrix targets = qValues;
    Matrix maxvalues = next_qValues.colwise().maxCoeff();

    // Q(s,a) = r + gamma * argmax_a Q(s', a)
    for(int i = 0; i < rewards.cols(); ++i ) {
        if(!isfinished[i])
            targets(actions(i), i) = rewards(i) + gamma * maxvalues(i);
        else
            targets(actions(i), i) = rewards(i);
    }
    return targets;
}

int DQN::generate_action(const Matrix& state)
{
    Matrix::Index action;

    // choose eps greedy action
    if( real_uni(rng) > epsilon ) {
        Vector qValues = net->predict(state);
        qValues.maxCoeff( &action );
    } else {
        // select complety random
        action = int_uni(rng);
    }
    return (int)action;
}

int DQN::generate_action(const std::vector<scalar_t>& state)
{
    const Vector state_m = Eigen::Map<const Vector, Eigen::Unaligned>(state.data(), state.size());
    return generate_action(state_m);
}

//-----------------------------------------------------------------------------------------------------------------------------------------

DDQN::DDQN(MiniDNN::Network* net, MiniDNN::Network* target_net, int batch_size, int swap_cnt, float epsilon, float alpha, float gamma )
    : DQN(net, batch_size, epsilon, alpha, gamma), target_net(target_net), swap_cnt(swap_cnt), cnt(0)
{
}

DDQN::DDQN(int batch_size, int swap_cnt, float epsilon, float alpha, float gamma )
    : DQN(batch_size, epsilon, alpha, gamma), target_net(NULL), swap_cnt(swap_cnt), cnt(0)
{
}

DDQN::~DDQN()
{
    delete target_net;
}

int DDQN::set_target_net(MiniDNN::Network* target_net)
{
    // target net input, ouput size must match net sizes
    if( target_net->in_size() != net->in_size() || target_net->out_size() != net->out_size() )
        return -1;

    this->target_net = target_net;
    return 0;
}

float DDQN::memory_replay()
{
    // 1. get samples form history buffer
    const Memory& samples = replay->sample();

    // Alternate between network and target network
    if(cnt == swap_cnt) {
        target_net->set_parameters( net->get_parameters() );
        cnt = 0;
    }
    cnt++;

    // 2. evaluate neural net at all states / next states
    Matrix qValues = net->predict( samples.states );                            // qValues of state
    Matrix next_qValues = net->predict( samples.next_states );
    Matrix next_qValues_t = target_net->predict( samples.next_states );         // qValues of next state based on target network

    // 3. compute target values
    Matrix targets = qtarget(samples.rewards, qValues, next_qValues, next_qValues_t, samples.actions, samples.isfinished, gamma);

    // 4. update network
    net->fit(opt, samples.states, targets, batch_size, 1, int_uni(rng)); // seed rng number
    return net->get_output()->loss();
}

Matrix DDQN::qtarget(const Matrix& rewards, const Matrix& qValues, const Matrix& next_qValues, const Matrix& next_qValues_t, const Matrix& actions, const std::vector<uint8_t>& isfinished, float gamma)
{
    Matrix targets = qValues;

    // Q(s,a) = r + gamma * Q(s', argmax_a Q_t(s', a) )
    int idx;
    for(int i = 0; i < rewards.cols(); ++i ) {
        if(!isfinished[i]) {
            next_qValues.col(i).maxCoeff(&idx); // index based on current
            targets(actions(i), i) = rewards(i) + gamma * next_qValues_t(idx, i); // update based on target
        }
        else
            targets(actions(i), i) = rewards(i);
    }
    return targets;
}

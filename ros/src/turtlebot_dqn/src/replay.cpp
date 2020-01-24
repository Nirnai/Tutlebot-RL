#include "turtlebot_dqn/replay.h"

using namespace Eigen;
using namespace dqn;

Replay::Replay(int buffer_size, int batch_size, int state_size)
    : buffer_size(buffer_size), batch_size(batch_size), state_size(state_size), cur_index(0), filled(false)
{
    memory.resize(buffer_size, state_size);
    sampled_memory.resize(batch_size, state_size);

    rng = std::mt19937(rd());
    int_uni = std::uniform_int_distribution<int>(0, buffer_size-1);
}

void Replay::initialize(int state_size)
{
    this->state_size = state_size;
    memory.resize(buffer_size, state_size);
    sampled_memory.resize(batch_size, state_size);
}

void Replay::initialize(const Memory& memory, int cur_index, bool filled )
{
    this->memory = memory;
    this->cur_index = cur_index;
    this->filled = filled;

    buffer_size = memory.get_buffer_size();
    state_size = memory.get_state_size();

    sampled_memory.resize(batch_size, state_size);
    int_uni = std::uniform_int_distribution<int>(0, buffer_size-1);
}

void Replay::push_back(const Matrix& state, const label_t& action, const scalar_t& reward, const Matrix& next_state,  const scalar_t& td, bool isfinished)
{
    if(cur_index >= buffer_size) {
        filled = true;
        cur_index = 0;
    }

    memory.states.col(cur_index) = state;
    memory.actions(0, cur_index) = action;
    memory.rewards(0, cur_index) = reward;
    memory.next_states.col(cur_index) = next_state;
    memory.td_errors(0, cur_index) = td;
    memory.isfinished[cur_index] = isfinished;

    cur_index++;
}

const Memory& Replay::sample()
{
    for( int i = 0; i < batch_size; ++i ) {
        int idx = int_uni(rng);

        sampled_memory.states.col(i) = memory.states.col(idx);
        sampled_memory.actions.col(i) = memory.actions.col(idx);
        sampled_memory.rewards.col(i) = memory.rewards.col(idx);
        sampled_memory.next_states.col(i) = memory.next_states.col(idx);
        sampled_memory.td_errors.col(i) = memory.td_errors.col(idx);
        sampled_memory.isfinished[i] = memory.isfinished[idx];
    }
    return sampled_memory;
}


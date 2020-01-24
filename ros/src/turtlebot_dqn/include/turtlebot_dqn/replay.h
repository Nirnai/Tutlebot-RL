#ifndef REPLAY_H
#define REPLAY_H

#include <eigen3/Eigen/Eigen>
#include <random>
#include <vector>

namespace dqn {

/* define type here */
typedef float scalar_t;
typedef size_t label_t;

typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> Matrix;
typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> Vector;

class Memory {
public:
    Matrix states;
    Matrix actions;
    Matrix rewards;
    Matrix next_states;
    Matrix td_errors;
    std::vector<uint8_t> isfinished;

    void resize(int buffer_size, int state_size) {
        states.resize(state_size, buffer_size);
        actions.resize(1, buffer_size);
        rewards.resize(1, buffer_size);
        next_states.resize(state_size, buffer_size);
        td_errors.resize(1, buffer_size);
        isfinished.resize(buffer_size);
    }

    int get_buffer_size() const { return states.cols(); }
    int get_state_size() const { return states.rows(); }
};

/* Replay buffer */
class Replay
{
public:
    Replay(int buffer_size, int batch_size, int state_size = 1);

    /* set new state size */
    void initialize(int state_size);
    void initialize(const Memory& memory, int cur_index, bool filled );

    /* buffer completly filled */
    bool is_filled() const { return filled; }

    /* return length of Replay */
    int get_buffer_size() const { return memory.get_state_size(); }

    /* return state dimension */
    int get_state_size() const { return memory.get_state_size(); }

    int get_batch_size() const { return batch_size; }

    /* add new Element to Replay */
    void push_back(const Matrix& state, const label_t& action, const scalar_t& reward, const Matrix& next_state,  const scalar_t& td, bool isfinished);

    /* return references to uniform sampled Replay elements */
    const Memory& sample();

    /* return reference to complete memory */
    const Memory& get_memory() const { return memory; }

    int get_current_index() const { return cur_index; }

private:
    bool filled;
    int cur_index;

    int buffer_size;                        // max buffer size
    int batch_size;
    int state_size;

    Memory memory;
    Memory sampled_memory;

    std::random_device rd;                  //  (seed) engine
    std::mt19937 rng;                       // random-number engine
    std::uniform_int_distribution<int> int_uni;
};

}

#endif // REPLAY_H

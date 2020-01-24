#include "turtlebot_dqn/turtlebot_lidar_dqn_env.h"

TurtlebotLidarDQNEnv::TurtlebotLidarDQNEnv() 
    : TurtlebotLidarEnv(), prev_distance(-1.0f), prev_theta(-1.0f)
{
}

TurtlebotLidarDQNEnv::~TurtlebotLidarDQNEnv()
{
}

/* returns the new observations */
bool TurtlebotLidarDQNEnv::observe(EnvState& env_state)
{
    // 1. call to base class (fill all protected state var)
    if( !TurtlebotLidarEnv::observe(env_state) )
        return false;

    // 2. normalize ranges to [0, 1]
    normalize_ranges(TurtlebotLidarEnv::ranges, normalized_ranges, TurtlebotLidarEnv::min_obstical_dist);

    // 3. compute reward
    if(prev_distance < 0.0) {
        prev_distance = TurtlebotLidarEnv::distance; // init step
        prev_theta = TurtlebotLidarEnv::theta;
    }

    float reward;
    if(hit_obstacle)
        reward = -150.0f;
    else if(reached_goal)
        reward = 100.0f;
    else {
        reward = std::min(1.5f, std::max(-1.5f, 15.0f * (prev_distance - distance) ) );
    }
    if(prev_action != 0) // rotation bad...
        reward -= 0.1f;

    // 4. combine into a single state vector ( scale state to range [0, 1] )
    auto env_state_t = static_cast<TypedEnvState<float>* >(&env_state);
    env_state_t->state = normalized_ranges;
    env_state_t->state.push_back( 1.0f / TurtlebotLidarEnv::distance * TurtlebotLidarEnv::min_goal_dist );
    env_state_t->state.push_back( TurtlebotLidarEnv::theta / M_PI + 1.0f );
    env_state_t->state.push_back( 1.0f / prev_distance * TurtlebotLidarEnv::min_goal_dist );
    env_state_t->state.push_back( prev_theta / M_PI + 1.0f );
    env_state_t->reward = reward;
    env_state_t->success = TurtlebotLidarEnv::reached_goal;
    env_state_t->done = TurtlebotLidarEnv::hit_obstacle | TurtlebotLidarEnv::reached_goal;

    prev_distance = TurtlebotLidarEnv::distance;
    prev_theta = TurtlebotLidarEnv::theta;
    return TurtlebotLidarEnv::is_goal_set;
}

void TurtlebotLidarDQNEnv::normalize_ranges(const std::vector<float>& ranges, std::vector<float>& normalized_ranges, float min_obstical_dist)
{
    normalized_ranges.resize(ranges.size());
    for( int i = 0; i < ranges.size(); ++i) {
        normalized_ranges[i] = 1.0f / ranges[i] * min_obstical_dist;
    }
}
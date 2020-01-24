#ifndef TURTLEBOTLIDARDQNENV_H_
#define TURTLEBOTLIDARDQNENV_H_

#include "turtlebot_dqn/turtlebot_lidar_env.h"

class TurtlebotLidarDQNEnv : public TurtlebotLidarEnv
{
public:
    TurtlebotLidarDQNEnv();
    virtual ~TurtlebotLidarDQNEnv();

    /* returns the new observations */
    virtual bool observe(EnvState& env_state);

private:
    float prev_distance;
    float prev_theta;

    std::vector<float> normalized_ranges;

    void normalize_ranges(const std::vector<float>& ranges, std::vector<float>& normalized_ranges, float min_obstical_dist);
};

#endif

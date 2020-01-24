#include "turtlebot_dqn/dqnlearner.h"
#include "turtlebot_dqn/utilities.h"
#include <fstream>

using namespace dqn;

DQNLearner::DQNLearner(AgentEnvironment *agentenv,  const std::string& log_dir, int net_save_steps)
    : agentenv(agentenv), ddqn( 256, 10000, 0.0f, 0.0004, 0.95 ), // bachsize 64, swap 1000, eps_init 0.98, alpha 0.0004, gamma 0.99
      log_dir(log_dir), net_save_steps(net_save_steps)
{
    MiniDNN::Network* net = build_network();
    MiniDNN::Network* target_net = build_network();
    Replay* replay = new Replay(1000, 128, net->in_size() );  // 3000000 TODO

    // 1. read weights if available
    std::ifstream ifs(log_dir+"net.dat", std::ifstream::in);
    if(ifs.is_open()) {
        std::cout << "Try Loading network weights" << std::endl;
        ifs >> *net;
        ifs >> *target_net;
        ifs.close();
        std::cout << "Loading network weights: n=" << net->in_size() * net->out_size() << std::endl;
    }
    // 2. read replay buffer
    ifs.open(log_dir+"replay.dat", std::ifstream::in);
    if(ifs.is_open()) {
        std::cout << "Try Loading replay buffer" << std::endl;
        ifs >> *replay;
        ifs.close();
        std::cout << "Loading replay buffer: size= " << replay->get_buffer_size() << std::endl;
    }

    // 3. pass networks and replay buffer to dqn
    ddqn.set_net(net);
    ddqn.set_target_net(target_net);
    ddqn.set_replay(replay);

    epsilon_discount = 0.9997;
}

DQNLearner::~DQNLearner()
{
    save();
}

MiniDNN::Network* DQNLearner::build_network()
{
    MiniDNN::Network* net = new MiniDNN::Network();
    MiniDNN::Layer* layer1 = new MiniDNN::FullyConnected<MiniDNN::ReLU>( agentenv->get_dim_states(), 256 );       // input dim statespace
    MiniDNN::Layer* layer2 = new MiniDNN::FullyConnected<MiniDNN::ReLU>( 256, 128 );
    MiniDNN::Layer* layer3 = new MiniDNN::FullyConnected<MiniDNN::Identity>( 128, agentenv->get_n_actions() );    // ouput dim actionspace
    net->add_layer(layer1);
    net->add_layer(layer2);
    net->add_layer(layer3);

    net->set_output(new MiniDNN::RegressionHuberloss(1.0f)); // MiniDNN::RegressionHuberloss(30.0f)
    net->init_normal(0.0f, 0.1f, 123); // random weights, seed 123

    return net;
}

/* start training session */
void DQNLearner::start(int total_episodes, int max_steps)
{
    this->total_episodes = total_episodes;
    this->max_steps = max_steps;

    rewards.reserve(total_episodes);
    fitting_errs.reserve(total_episodes);
    epsilons.reserve(total_episodes);
    selected_actions.reserve(total_episodes);

    q_value_start.resize( agentenv->get_n_actions() );
    q_value_goal.resize( agentenv->get_n_actions() );
    for(int i = 0; i < agentenv->get_n_actions(); ++i) {
        q_value_start.reserve( total_episodes );
        q_value_goal.reserve( total_episodes );
    }

    cnt_episodes = 0;
    cnt_steps = 0;
    cumulated_reward = 0;
    is_started = true;

    agentenv->reset();
    is_after_reset = true;
}

/* main update loop call this function as fast as possible */
int DQNLearner::update(bool user_action)
{
    if(!is_started )
        return -1;  // not started
    if(cnt_episodes > total_episodes) {
        std::cout << "Finishes after: " << cnt_episodes << std::endl;
        return 1;   // finished
    }

    // simulation first iteration ?
    if(is_after_reset) {
        is_after_reset = false;
        // get inital observation
        agentenv->observe(env_state);
        state = env_state.state;
        env_inital_state = env_state.state;

        // get inital action
        action = ddqn.generate_action( state );
    }

    // 1. do observation (if not available, repeat the same action again, no learning)
    if( !agentenv->observe(env_state) ) {
        agentenv->execute( action );
        return 0;
    }

    // 2. learn
    // float err = ddqn.learn(state, action, env_state.reward, env_state.state, env_state.done);
    float err = 0.0f;

    // 3. transition
    state = env_state.state;
    cumulated_reward += env_state.reward;
    cnt_steps++;

    // 4. execute next action
    action = ddqn.generate_action( state );
    agentenv->execute( action );

    // check if episode finished
    if( env_state.done ) {
        std::cout << "EP: " << cnt_episodes << " steps: " << cnt_steps <<  " eps: " << ddqn.getEpsilon()
                  << " rew: " << cumulated_reward << " net_err:" << err << std::endl;

        // monitor variables
        rewards.push_back(cumulated_reward);
        fitting_errs.push_back(err);
        epsilons.push_back(ddqn.getEpsilon());
        num_of_steps.push_back(cnt_steps);
        selected_actions.push_back(action);

        q_value_start[0].push_back( ddqn.getQ(env_inital_state, 0) );
        q_value_start[1].push_back( ddqn.getQ(env_inital_state, 1) );
        q_value_start[2].push_back( ddqn.getQ(env_inital_state, 2) );
        if( env_state.success ) {
            q_value_goal[0].push_back( ddqn.getQ(state, 0) );
            q_value_goal[1].push_back( ddqn.getQ(state, 1) );
            q_value_goal[2].push_back( ddqn.getQ(state, 2) );
        }

        // reset counter
        cnt_steps = 0;
        cumulated_reward = 0;

        std::cout << "DQNLearner::update loop set monitor var" << std::endl;

        // update eps
        if( ddqn.getEpsilon() > 0.15 && err != 0.0f)
            ddqn.setEpsilon( ddqn.getEpsilon() * epsilon_discount );

        cnt_episodes++;
        agentenv->reset();
        is_after_reset = true;

        std::cout << "DQNLearner::update loop set epsilon var" << std::endl;

        // save net weights if net_save_steps reached
        if( cnt_episodes % net_save_steps == 0) {
            std::string file_name = log_dir + "net_" + std::to_string(cnt_episodes) + ".dat";
            std::ofstream ofs(file_name, std::ofstream::out | std::ostream::binary);
            ofs << *ddqn.get_net();
            ofs << *ddqn.get_target_net();
            ofs.close();
        }

        std::cout << "DQNLearner::update loop saved net" << std::endl;
    }
    return 0;
}

int DQNLearner::save()
{
    // 1. store net and target net
    std::ofstream ofs(log_dir+"net.dat", std::ofstream::out | std::ostream::binary);
    ofs << *ddqn.get_net();
    ofs << *ddqn.get_target_net();
    ofs.close();

    // 2. store replay buffer
    ofs.open(log_dir+"replay.dat", std::ofstream::out | std::ostream::binary);
    ofs << *ddqn.get_replay();
    ofs.close();

    // 2. store envrionment information in mat file
    matio::MatStruct monitor;
    monitor.add("cumulated_reward", rewards);
    monitor.add("fitting_errs", fitting_errs);
    monitor.add("num_of_steps", num_of_steps);
    monitor.add("epsilons", epsilons);
    monitor.add("selected_actions", selected_actions);

    monitor.add("q_value_start_1", q_value_start[0]);
    monitor.add("q_value_start_2", q_value_start[1]);
    monitor.add("q_value_start_3", q_value_start[2]);

    monitor.add("q_value_goal_1", q_value_goal[0]);
    monitor.add("q_value_goal_2", q_value_goal[1]);
    monitor.add("q_value_goal_3", q_value_goal[2]);

    monitor.write(log_dir+"environment_log.mat", "env_struct");
}

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <vector>
#include <numeric>
#include <algorithm>
#include <math.h>
#include <geometry_msgs/Pose2D.h>

#include "MiniDNN/MiniDNN.h"
#include "turtlebot_dqn/replay.h"

namespace geom2d 
{
	using namespace geometry_msgs;

	/* squared Distance point to point */
	inline double distance_point_sq(const Pose2D& p1, const Pose2D& p2) {
		return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
	}

	/* Distance point to point */
	inline double distance_point(const Pose2D& p1, const Pose2D& p2) {
		return sqrt( distance_point_sq(p1, p2) );
	}
}

/*
 * Function to store qlearning to binary file
 * overload << , >> operator for class qlearn
 */
namespace dqn {

    /* write std vector to stream */
    template<typename T>
    inline std::ostream& operator<< (std::ostream& out, const std::vector<T>& vec)
    {
        size_t size = vec.size();

        out.write((char*)(&size), sizeof(size_t));
        out.write((char*)vec.data(), size * sizeof(T));
        return out;
    }
    /* read std vector form stream */
    template<typename T>
    std::istream& operator>> (std::istream& in, std::vector<T>& vec)
    {
        size_t size = 0;
        in.read((char*)(&size), sizeof(size_t));
        vec.resize(size);
        in.read((char*)vec.data(), size*sizeof(T));
        return in;
    }

    /* write Eigen matrix to stream */
    inline std::ostream& operator<< (std::ostream& out, const Matrix& matrix)
    {
        typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();

        out.write((char*)(&rows), sizeof(typename Matrix::Index));
        out.write((char*)(&cols), sizeof(typename Matrix::Index));
        out.write((char*)matrix.data(), rows*cols*sizeof(typename Matrix::Scalar));
        return out;
    }
    /* read Eigen matrix form stream */
    inline std::istream& operator>> (std::istream& in, Matrix& matrix)
    {
        typename Matrix::Index rows=0, cols=0;

        in.read((char*)(&rows), sizeof(typename Matrix::Index));
        in.read((char*)(&cols), sizeof(typename Matrix::Index));
        matrix.resize(rows, cols);
        in.read((char*)matrix.data(), rows*cols*sizeof(typename Matrix::Scalar));
        return in;
    }

    /* write MiniDNN::Network weights to stream */
    inline std::ostream& operator<< (std::ostream &out, const Replay& replay)
    {
        size_t buffer_size=0, state_size=0, filled=0, cur_index=0;
        auto memory = replay.get_memory();
        buffer_size = memory.get_buffer_size();
        state_size = memory.get_state_size();
        filled = replay.is_filled();
        cur_index = replay.get_current_index();

        out.write((char*)(&buffer_size), sizeof(size_t));
        out.write((char*)(&state_size), sizeof(size_t));
        out.write((char*)(&filled), sizeof(size_t));
        out.write((char*)(&cur_index), sizeof(size_t));

        out << memory.states;
        out << memory.actions;
        out << memory.rewards;
        out << memory.next_states;
        out << memory.td_errors;
        out << memory.isfinished;

        return out;
    }
    /* read MiniDNN::Network weights form stream */
    inline std::istream& operator>> (std::istream &in, Replay& replay)
    {
        size_t buffer_size=0, state_size=0, filled=0, cur_index=0;
        Memory memory;

        in.read((char*)(&buffer_size), sizeof(size_t));
        in.read((char*)(&state_size), sizeof(size_t));
        in.read((char*)(&filled), sizeof(size_t));
        in.read((char*)(&cur_index), sizeof(size_t));
        memory.resize(buffer_size, state_size);

        in >> memory.states;
        in >> memory.actions;
        in >> memory.rewards;
        in >> memory.next_states;
        in >> memory.td_errors;
        in >> memory.isfinished;

        replay.initialize(memory, cur_index, filled);
        return in;
    }

    /* write MiniDNN::Network weights to stream */
    inline std::ostream& operator<< (std::ostream &out, const MiniDNN::Network& network)
    {
        size_t size = 0;
        auto weights = network.get_parameters();
        size = weights.size();

        out.write((char*)(&size), sizeof(size_t));
        for(size_t i = 0; i < size; ++i) {
            out << weights[i];
        }
        return out;
    }
    /* read MiniDNN::Network weights form stream */
    inline std::istream& operator>> (std::istream &in, MiniDNN::Network& network)
    {
        size_t size = 0;
        in.read((char*)(&size), sizeof(size_t));
        std::vector< std::vector<MiniDNN::Scalar> > weights(size);
        for(size_t i = 0; i < size; ++i) {
            in >> weights[i];
        }
        network.set_parameters(weights);
        return in;
    }
}

#endif

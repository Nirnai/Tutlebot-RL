#ifndef OUTPUT_REGRESSIONHUBERLOSS_H_
#define OUTPUT_REGRESSIONHUBERLOSS_H_

#include <eigen3/Eigen/Core>
#include <stdexcept>
#include "../Config.h"

namespace MiniDNN {

///
/// \ingroup Outputs
///
/// Regression output layer using Huber loss (more robust to outlieres)
///
class RegressionHuberloss: public Output
{
private:
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;

    Matrix m_din;  // Derivative of the input of this layer.
                   // Note that input of this layer is also the output of previous layer

    float delta;   // intervall to use mse, outside linear

public:
    RegressionHuberloss(float delta = 1.0f) : delta(delta) {}

    void evaluate(const Matrix& prev_layer_data, const Matrix& target)
    {
        // Check dimension
        const int nobs = prev_layer_data.cols();
        const int nvar = prev_layer_data.rows();
        if((target.cols() != nobs) || (target.rows() != nvar))
            throw std::invalid_argument("[class RegressionMSE]: Target data have incorrect dimension");

        // Compute the derivative of the input of this layer
        // L = 0.5*||yhat - y||^2                           for |x| <= delta
        // L = delta * ||yhat - y||-delta + 0.5 delta^2     for |x| >  delta
        // in = yhat
        // d(L) / d(in) = (yhat - y)    for |x| <= delta
        // d(L) / d(in) = +- delta        for |x| <= delta
        m_din.resize(nvar, nobs);
        m_din.noalias() = prev_layer_data - target; // (yhat - y)
        m_din = m_din.array().min(delta).max(-delta); // clip to [-1.0, 1.0]
    }

    const Matrix& backprop_data() const
    {
        return m_din;
    }

    Scalar loss() const
    {
        // L = 0.5 * ||yhat - y||^2 ... wrong
        return m_din.squaredNorm() / m_din.cols() * Scalar(0.5); // ??
    }
};


} // namespace MiniDNN

#endif // HUBERLOSS_H

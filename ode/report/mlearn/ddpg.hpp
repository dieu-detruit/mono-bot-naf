#pragma once

#include <Eigen/Core>

#include <array>
#include <iostream>

namespace DQN
{

class Q
{
private:
    Eigen::Matrix<double, 3, 16> weight_1;
    Eigen::Matrix<double, 16, 64> weight_2;
    Eigen::Matrix<double, 64, 64> weight_3;
    Eigen::Matrix<double, 64, 1> weight_4;

public:
    double ReLu(double x)
    {
        return (x < 0) ? 0 : x;
    }
    double dReLu(double x)
    {
        return (x < 0) ? 0 : 1;
    }
    template <int n>
    Eigen::Matrix<double, n, 1> ReLu(const Eigen::Matrix<double, n, 1>& x)
    {
        Eigen::Matrix<double, n, 1> y;
        for (int i = 0; i < n; ++i) {
            y(i, 0) = ReLu(x(i, 0));
        }
        return y;
    }
    template <int n>
    Eigen::Matrix<double, n, 1> dReLu(const Eigen::Matrix<double, n, 1>& x)
    {
        Eigen::Matrix<double, n, 1> y;
        for (int i = 0; i < n; ++i) {
            y(i, 0) = dReLu(x(i, 0));
        }
        return y;
    }


    std::array<double, 2> predict(std::array<double, 2> state, int action_ankle, int action_knee)
    {
        Eigen::Vector4d x{state.at(0), state.at(1), action_ankle, action_knee};

        Eigen::Matrix<double, 16, 1> y1 = ReLu(weight_1 * x);
        Eigen::Matrix<double, 64, 1> y2 = ReLu(weight_2 * y1);
        Eigen::Matrix<double, 64, 1> y3 = ReLu(weight_3 * y2);
        Eigen::Matrix<double, 1, 1> y4 = ReLu(weight_4 * y3);
    }
};

class J
{
};

class Agent
{
private:
    double gamma = 0.8;     // 割引率
    double epsilon = 0.99;  // 乱択率

public:
    Agent(const double gamma, const double epsilon)
        : gamma(gamma), epsilon(epsilon)
    {
    }
    ~Agent() = default;
};

}  // namespace DQN

#include <drawstuff/drawstuff.h>  // 描画用ヘッダーファイル
#include <ode/ode.h>              // ODE用ヘッダーファイル

#include "simulation.hpp"

#include <Eigen/Core>

#include <cmath>
#include <fstream>
#include <iostream>

/*
std::array<dReal, 2> user_callback(const std::array<dReal, 2>& state)
{
    Eigen::Vector2d state_vec{state.at(0), state.at(1)};

    std::array<dReal, 2> input;
    input.at(0) = 0;
    input.at(1) = 0;

    return input;
}
*/

/*
std::array<dReal, 2> user_callback(const std::array<dReal, 2>& state)
{
    std::array<dReal, 2> input;
    static std::array<dReal, 2> _state = state;

    if (state.at(0) < 0) {
        input.at(0) = 80;
    } else {
        input.at(0) = -80;
    }
    std::cout << "ankle = " << state.at(0) << std::endl;
    std::cout << "knee = " << state.at(1) << std::endl;
    std::cout << input.at(0) << std::endl;
    input.at(1) = 0;

    _state = state;

    return input;
}
*/

double kp_0 = 0.0, kv_0 = 0.0;
double kp_1 = 0.0, kv_1 = 0.0;
void loadParams()
{
    std::ifstream file("param.txt");
    file >> kp_0 >> kv_0 >> kp_1 >> kv_1;
    std::cout << "params : " << kp_0 << kv_0 << kp_1 << kv_1 << std::endl;
}

std::array<dReal, 2> user_callback(const std::array<dReal, 2>& state)
{
    static double t = 0.0;

    std::array<dReal, 2> input;
    static std::array<dReal, 2> _state = state;

    {
        std::cout << "ankle = " << state.at(0) << std::endl;
        input.at(0) = kp_0 * (0.1 - state.at(0)) + kv_0 * (_state.at(0) - state.at(0));
        std::cout << input.at(0) << std::endl;
    }

    {
        std::cout << "knee = " << state.at(1) << std::endl;
        input.at(1) = kp_0 * (-(0.4 * M_PI * std::sin(M_PI * t) + 0.1 * M_PI) - state.at(0)) + kv_0 * (-0.4 * M_PI * M_PI * std::cos(M_PI * t) - (_state.at(0) - state.at(0)));
        std::cout << input.at(1) << std::endl;
    }

    _state = state;
    t += 0.05;

    return input;
}

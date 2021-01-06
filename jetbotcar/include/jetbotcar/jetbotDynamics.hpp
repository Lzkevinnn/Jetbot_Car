#pragma once

#include "jetbotcar/car_state.hpp"
#include "jetbotcar/car_params.hpp"

namespace racecar_simulator {


class lowPassFilter {
public:
    double& timeConstant; // time to 90% of ss response
    const double& state;
    double update(double dt, double input);
    lowPassFilter(double&, double&);
};
class jetbotKinematics {
public:
    static twoWheelBotState kinematicUpdate(
        const twoWheelBotState initialState,
        double  rightWheelSpeed,
        double  leftWheelSpeed,
        twoWheelBotParameters carParameters,
        double dt);
};
}
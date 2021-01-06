#include <cmath>

#include "jetbotcar/car_state.hpp"
#include "jetbotcar/jetbotDynamics.hpp"
#include <iostream>

using namespace racecar_simulator;



    
double lowPassFilter::update(double dt, double input) {
    double filterOutput;
    double filterCoefficient = timeConstant/2.3;
    filterOutput = state * (filterCoefficient / (dt + filterCoefficient)) + input * (dt / (dt + filterCoefficient));
    return filterOutput;
}
lowPassFilter::lowPassFilter(double& riseTime, double& filterState): timeConstant(riseTime), state(filterState){}
twoWheelBotState jetbotKinematics::kinematicUpdate(
    const twoWheelBotState initialState,
    double  rightWheelSpeed,
    double  leftWheelSpeed,
    twoWheelBotParameters carParameters,
    double dt) {
    twoWheelBotState finalState;
    double rightLinearWheelSpeed = carParameters.wheelRadius * rightWheelSpeed;
    double leftLinearWheelSpeed = carParameters.wheelRadius * leftWheelSpeed;
    finalState.leftWheelSpeed = leftWheelSpeed;
    finalState.rightWheelSpeed = rightWheelSpeed;
    finalState.angular_velocity = (rightLinearWheelSpeed - leftLinearWheelSpeed) / (carParameters.track);
    finalState.velocity = (rightLinearWheelSpeed + leftLinearWheelSpeed) / 2;
    finalState.theta = initialState.theta + finalState.angular_velocity * dt;
    finalState.x = initialState.x + std::cos(finalState.theta) * finalState.velocity * dt;
    finalState.y = initialState.y + std::sin(finalState.theta) * finalState.velocity * dt;
    finalState.std_dyn = false;
    return finalState;
    
}



 
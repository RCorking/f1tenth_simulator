#include <cmath>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/jetbotDynamics.hpp"
#include <iostream>

using namespace racecar_simulator;



    
double lowPassFilter::update(double dt, double input) {
    double filterOutput;
    filterOutput = *state * (filterCoefficient / (dt + filterCoefficient)) + input * (dt / (dt + filterCoefficient));
    return filterOutput;
}
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



 
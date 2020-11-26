#include <cmath>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/jetbotDynamics.hpp"
#include <iostream>

using namespace racecar_simulator;


twoWheelBotState jetbotDynamics::update(
    const twoWheelBotState initialState,
    double rightWheelTorque,
    double leftWheelTorque,
    twoWheelBotParameters carParameters,
    const double thresh,
    double dt) {

    double err = 0.3; // deadband to avoid flip flop 

    twoWheelBotState finalState;
    double rightWheelForce, leftWheelForce, forwardAcceleration, angularAcceleration, updatedLinearVelocity, updatedAngularVelocity, instatTurnRadius,
        updatedRightWheelSpeed, updatedLeftWheelSpeed;
        rightWheelForce = carParameters.wheelRadius * rightWheelTorque - carParameters.wheelDampingFactor * initialState.rightWheelSpeed;
        leftWheelForce = carParameters.wheelRadius * leftWheelTorque - carParameters.wheelDampingFactor * initialState.leftWheelSpeed;
        forwardAcceleration = (rightWheelForce + leftWheelForce) / carParameters.mass;
        angularAcceleration = (rightWheelForce - leftWheelForce) * (carParameters.track) / (2.0);
        updatedLinearVelocity = initialState.velocity + dt * forwardAcceleration;
        updatedAngularVelocity = initialState.angular_velocity + dt * angularAcceleration;
        instatTurnRadius;
        updatedRightWheelSpeed;
        updatedLeftWheelSpeed;
        if (abs(updatedAngularVelocity) >= thresh) {
            instatTurnRadius = updatedLinearVelocity / updatedAngularVelocity;
            updatedRightWheelSpeed = ((instatTurnRadius + carParameters.track / 2) / (instatTurnRadius)) *
                updatedLinearVelocity / carParameters.wheelRadius;
            updatedLeftWheelSpeed = ((instatTurnRadius - carParameters.track / 2) / instatTurnRadius) *
                updatedLinearVelocity / carParameters.wheelRadius;
        }
        else {
            instatTurnRadius = 0;
            updatedRightWheelSpeed = updatedLinearVelocity / carParameters.wheelRadius;
            updatedLeftWheelSpeed = updatedLinearVelocity / carParameters.wheelRadius;
        }
    
    double updatedHeading = initialState.theta + dt * updatedAngularVelocity;
    double updatedX = initialState.x + std::cos(initialState.theta) * updatedLinearVelocity * dt;
    double udpatedY = initialState.y + std::sin(initialState.theta) * updatedLinearVelocity * dt;
        finalState = {  .x = updatedX,
                        .y = udpatedY,
                        .theta = updatedHeading,
                        .velocity = updatedLinearVelocity,
                        .angular_velocity = updatedAngularVelocity,
                        .leftWheelSpeed = updatedLeftWheelSpeed,
                        .rightWheelSpeed = updatedRightWheelSpeed,
                        .std_dyn = false
        };
        return finalState;    
}
//lowPassFilter::lowPassFilter(double timeConst, double& filterState) :  state(filterState) {
//    filterCoefficient = -timeConst / (2.3);
//}
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



 
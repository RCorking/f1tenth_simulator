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
    double dt,
    bool dynamicSim) {

    double err = 0.3; // deadband to avoid flip flop 

    twoWheelBotState finalState;
    double rightWheelForce, leftWheelForce, forwardAcceleration, angularAcceleration, updatedLinearVelocity, updatedAngularVelocity, instatTurnRadius,
        updatedRightWheelSpeed, updatedLeftWheelSpeed;
    if (dynamicSim) {
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
    }
    else {



    }
    double updatedHeading = initialState.theta + dt * updatedAngularVelocity;
    double updatedX = initialState.x + std::cos(initialState.theta) * updatedLinearVelocity * dt;
    double udpatedY = initialState.y + std::sin(initialState.theta) * updatedLinearVelocity * dt;

        finalState = { .x = updatedX,
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
lowPassFilter::lowPassFilter(double timeStep, double timeConst, double& filterState) : dt(timeStep), timeConstant(timeConst), state(filterState) {}
  

 double lowPassFilter::filterUpdate(double input) {
     double filterCoefficient = -timeConstant / (2.3);
     double filterOutput;
     filterOutput = state * (filterCoefficient / (dt + filterCoefficient)) + input * (dt / (dt + filterCoefficient));
     return filterOutput;
 }




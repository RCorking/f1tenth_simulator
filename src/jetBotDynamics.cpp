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
            double dt){

        double err =0.3 ; // deadband to avoid flip flop 

        twoWheelBotState finalState;

        double rightWheelForce = carParameters.wheelRadius*rightWheelTorque - carParameters.wheelDampingFactor*initialState.rightWheelSpeed;
        double leftWheelForce = carParameters.wheelRadius*leftWheelTorque - carParameters.wheelDampingFactor*initialState.leftWheelSpeed ;
        double forwardAcceleration = (rightWheelForce + leftWheelForce)/carParameters.mass;
        double angularAcceleration = (rightWheelForce - leftWheelForce)*(carParameters.track)/(2.0);
        double updatedLinearVelocity = initialState.velocity + dt*forwardAcceleration;
        double updatedAngularVelocity = initialState.angular_velocity + dt*angularAcceleration;
        double updatedHeading = initialState.theta + dt*updatedAngularVelocity;
        double updatedX = initialState.x + std::cos(initialState.theta)*updatedLinearVelocity*dt;
        double udpatedY = initialState.y + std::sin(initialState.theta)*updatedLinearVelocity*dt;
        double instatTurnRadius;
        double updatedRightWheelSpeed;
        double updatedLeftWheelSpeed;
        if (abs(updatedAngularVelocity) >= thresh){
            instatTurnRadius = updatedLinearVelocity/updatedAngularVelocity;
            updatedRightWheelSpeed = ((instatTurnRadius + carParameters.track/2)/(instatTurnRadius))*
                                               updatedLinearVelocity/carParameters.wheelRadius;
            updatedLeftWheelSpeed = ((instatTurnRadius - carParameters.track/2)/instatTurnRadius)*
                                                updatedLinearVelocity/carParameters.wheelRadius;
            

        }else{
            instatTurnRadius = 0;
            updatedRightWheelSpeed = updatedLinearVelocity/carParameters.wheelRadius;
            updatedLeftWheelSpeed = updatedLinearVelocity/carParameters.wheelRadius;
        }

        finalState = {  .x = updatedX,
                        .y = udpatedY,
                        .theta = updatedHeading,
                        .velocity = updatedLinearVelocity,
                        .angular_velocity = updatedAngularVelocity,
                        .leftWheelSpeed = updatedLeftWheelSpeed,
                        .rightWheelSpeed = updatedRightWheelSpeed,
                        .std_dyn= false 
                      };


        return finalState;


        }




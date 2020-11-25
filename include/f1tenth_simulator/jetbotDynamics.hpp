#pragma once

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"

namespace racecar_simulator {

class jetbotDynamics{
    public:
        static twoWheelBotState update(
            const twoWheelBotState initialState,
            double rightWheelTorque,
            double leftWheelTorque,
            twoWheelBotParameters carParameters,
            const double thresh,
            double dt);
    
};
    double timeConstant; // time to 90% of ss response
    double filterCoefficient; // time to 90% of ss response
    const double& state;
    double update(double dt, double input);
    lowPassFilter(double, double&);
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
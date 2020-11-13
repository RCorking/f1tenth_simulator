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
}
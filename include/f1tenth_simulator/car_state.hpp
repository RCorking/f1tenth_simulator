#pragma once

namespace racecar_simulator {

struct CarState {
    double x; // x position
    double y; // y position
    double theta; // orientation
    double velocity;
    double steer_angle;
    double angular_velocity;
    double slip_angle;
    bool st_dyn;
};

struct twoWheelBotState {
    double x;
    double y;
    double theta;
    double velocity;
    double angular_velocity;
    double leftWheelSpeed;
    double rightWheelSpeed;
    bool std_dyn;
};

}

/**
 * @file    MyRobot.cpp
 * @brief   cpp file practice 4 (straight_odometry). Definition of all the code for the correct operation of the robot
 *
 * @author  Luis A. Martín <100278361@alumnos.uc3m.es>
 * @date    2014-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    //Distance traveled the robot to the target
    desired_distance=17;

    enableEncoders(_time_step);

    //Initializing variables
    _left_speed = 100;
    _right_speed = 100;
    _distance = 0;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    disableEncoders();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    while (step(_time_step) != -1)
    {
        //Encoders read and assign to variables
        _left_encoder = getLeftEncoder();
        _right_encoder = getRightEncoder();

        //1 meter = encoder reading/60.61 (valid for resolution of enconder in simulation of 5)
        _distance = _left_encoder/60.61;

        //Assigning values ​​to the corresponding variables read
        cout<< " ESTIMATED DISTANCE:   "<<_distance<< endl;




        //Control logic

        if (_distance < desired_distance)
        {
            if(_left_encoder > _right_encoder)
            {
                _left_speed = 90;
                _right_speed = 100;
            }
            else
            {
                _left_speed = 100;
                _right_speed = 90;
            }
        }
        else
        {
            _left_speed = 0;
            _right_speed = 0;
        }
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////


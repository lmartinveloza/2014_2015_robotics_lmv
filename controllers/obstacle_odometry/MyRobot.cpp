/**
 * @file    MyRobot.cpp
 * @brief   description of all the code necessary for the proper practice 4 (obstacle_odometry)
 *
 * @author  Luis A. Martín <100278361@alumnos.uc3m.es>
 * @date    2014-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    //Habilitation of enconders
    enableEncoders(_time_step);

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    //Initialization of variables
    desired_distance=0;
    desired_angle=0;
    _left_speed = 0;
    _right_speed = 0;
    _distance = 0;
    _status=0;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    //Disconnection of the encoders and the compass
    disableEncoders();
    _my_compass->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;

    while (step(_time_step) != -1)
    {

        //Control and description of the states of the robot

        switch (_status)
        {
        case 0:
            desired_angle=85;
            break;
        case 1:
            desired_distance=5.50;
            desired_angle=85;
            break;
        case 2:
            desired_distance=0;
            desired_angle=45;
            break;
        case 3:
            desired_distance=12.10;
            desired_angle=45;
            break;
        default:
            desired_distance=0;
            desired_angle=0;
            break;
        }

        // Read the compass
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        //Assigning values ​​to the corresponding variables read
        _left_encoder = getLeftEncoder();
        _right_encoder = getRightEncoder();
        _distance = _left_encoder/60.61;

        //Assigning values ​​to the corresponding variables read
        cout<< " ESTIMATED DISTANCE:   "<<_distance<< endl;
        cout<< " ANGLE FOLLOWING:    "<< compass_angle<<endl;
        cout<< " BEEN ACTIVATED:   "<< _status<< endl;

        //Logical control of the robot
        if (_status<4)
        {
            if (((compass_angle<desired_angle-1)||(compass_angle>desired_angle+1)) && (_status==0))
            {
                _left_speed = 10;
                _right_speed = 0;
                //Reset the encoder and distance values for next reading
                setEncoders(0,0);
                _distance = 0;
            }
            else if (((compass_angle<desired_angle-1)||(compass_angle>desired_angle+1)) && (_status==2))
            {
                _left_speed = 0;
                _right_speed = 10;
                //Reset the encoder and distance values for next reading
                setEncoders(0,0);
                _distance = 0;
            }
            else if (_distance < desired_distance)
            {
                if(_left_encoder > _right_encoder)
                {
                    _left_speed = MAX_SPEED-10;
                    _right_speed = MAX_SPEED;
                }
                else
                {
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED-10;
                }
            }
            else
            {
                _left_speed = 0;
                _right_speed = 0;
                //Reset the encoder and distance values for next reading
                setEncoders(0,0);
                _distance = 0;
                //Change in state
                _status ++;
            }
        }
        //Assigned to the robot velocity values
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    //Function for conversion to degrees
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////

/**
 * @file    MyRobot.cpp
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date    2014-07
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    //enable the standard mode
    _mode = forward;

    //Get and enable the sensors device

    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds13");
    _distance_sensor[3]->enable(_time_step);
    _distance_sensor[4] = getDistanceSensor("ds14");
    _distance_sensor[4]->enable(_time_step);
    _distance_sensor[5] = getDistanceSensor("ds15");
    _distance_sensor[5]->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double ir0_val = 0.0, ir1_val = 0.0, ir2_val = 0.0, ir13_val=0.0, ir14_val=0.0, ir15_val=0.0;

    while (step(_time_step) != -1) {
        // Read the sensors
        ir0_val = _distance_sensor[0]->getValue();
        ir1_val = _distance_sensor[1]->getValue();
        ir2_val = _distance_sensor[2]->getValue();
        ir13_val=_distance_sensor[3]->getValue();
        ir14_val=_distance_sensor[4]->getValue();
        ir15_val=_distance_sensor[5]->getValue();

        // Print sensor values to console
        cout << "ds0(front): " << ir0_val <<endl<< " ds15(front):" << ir15_val << endl;
        cout << "ds1(left): " << ir1_val <<endl<< " ds2(left):" << ir2_val << endl;
        cout << "ds13(right): " << ir13_val <<endl<< " ds14(right):" << ir14_val << endl;



        // Control logic of the robot
        if (_mode == forward) {


            //Read a close front wall, change to following mode
            if ((ir0_val > DISTANCE_LIMIT) || (ir1_val > DISTANCE_LIMIT) || (ir14_val > DISTANCE_LIMIT) || (ir15_val > DISTANCE_LIMIT) ) {
                _mode = follower;
                cout << "MODE " << follower << ": WALL FOLLOWING" << endl;
            }
        }
        else {
            // Wall following

            if ((ir0_val > DISTANCE_LIMIT) || (ir1_val > DISTANCE_LIMIT) || (ir14_val > DISTANCE_LIMIT) || (ir15_val > DISTANCE_LIMIT)) {
                _mode = follower;
                cout << "BACK & TURN LEFT." << endl;
            }
            else {
                if (ir13_val > DISTANCE_LIMIT) {
                    _mode = left;
                    cout << "MOVING TO THE LEFT." << endl;
                }
                else {
                    if (ir13_val < DISTANCE_LIMIT + 50) {
                        _mode = right;
                        cout << "MOVING TO THE RIGHT." << endl;
                    }
                    else {
                        _mode = forward;
                        cout << "MOVING FORWARD." << endl;
                    }
                }
            }
        }

        // Operating modes:
        switch (_mode){

            case left:
                _left_speed = MAX_SPEED / 1.50;
                _right_speed = MAX_SPEED;
                break;

            case right:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED / 1.50;
                break;

            case follower:
                _left_speed = -MAX_SPEED / 2.0;
                _right_speed = -MAX_SPEED / 15.0;
                break;

             case forward:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
            break;

            case stop:
                _left_speed = 0;
                _right_speed = 0;
            break;

            default:
                break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

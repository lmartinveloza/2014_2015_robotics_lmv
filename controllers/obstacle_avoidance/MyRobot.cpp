/**
 * @file    main_template.cpp
 * @brief   A template for webots projects.
 *
 * @author  Name Surname <nick@alumnos.uc3m.es>
 * @date    2014-07
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    _mode = FORWARD;

    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds3");
    _distance_sensor[3]->enable(_time_step);
    _distance_sensor[4] = getDistanceSensor("ds4");
    _distance_sensor[4]->enable(_time_step);
    _distance_sensor[5] = getDistanceSensor("ds5");
    _distance_sensor[5]->enable(_time_step);
    _distance_sensor[6] = getDistanceSensor("ds6");
    _distance_sensor[6]->enable(_time_step);
    _distance_sensor[7] = getDistanceSensor("ds7");
    _distance_sensor[7]->enable(_time_step);
    _distance_sensor[8] = getDistanceSensor("ds8");
    _distance_sensor[8]->enable(_time_step);
    _distance_sensor[9] = getDistanceSensor("ds9");
    _distance_sensor[9]->enable(_time_step);
    _distance_sensor[10] = getDistanceSensor("ds10");
    _distance_sensor[10]->enable(_time_step);
    _distance_sensor[11] = getDistanceSensor("ds11");
    _distance_sensor[11]->enable(_time_step);
    _distance_sensor[12] = getDistanceSensor("ds12");
    _distance_sensor[12]->enable(_time_step);
    _distance_sensor[13] = getDistanceSensor("ds13");
    _distance_sensor[13]->enable(_time_step);
    _distance_sensor[14] = getDistanceSensor("ds14");
    _distance_sensor[14]->enable(_time_step);
    _distance_sensor[15] = getDistanceSensor("ds15");
    _distance_sensor[15]->enable(_time_step);
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
    double ir0_val = 0.0, ir1_val = 0.0, ir2_val = 0.0, ir3_val = 0.0, ir4_val = 0.0;
    double ir5_val = 0.0, ir6_val = 0.0, ir7_val = 0.0, ir8_val = 0.0, ir9_val = 0.0;
    double ir10_val = 0.0, ir11_val = 0.0, ir12_val = 0.0, ir13_val = 0.0, ir14_val = 0.0, ir15_val=0.0;

    while (step(_time_step) != -1) {
        // Read the sensors
        ir0_val = _distance_sensor[0]->getValue();
        ir1_val = _distance_sensor[1]->getValue();
        ir2_val = _distance_sensor[2]->getValue();
        ir3_val = _distance_sensor[3]->getValue();
        ir4_val = _distance_sensor[4]->getValue();
        ir5_val = _distance_sensor[5]->getValue();
        ir6_val = _distance_sensor[6]->getValue();
        ir7_val = _distance_sensor[7]->getValue();
        ir8_val = _distance_sensor[8]->getValue();
        ir9_val = _distance_sensor[9]->getValue();
        ir10_val = _distance_sensor[10]->getValue();
        ir11_val = _distance_sensor[11]->getValue();
        ir12_val = _distance_sensor[12]->getValue();
        ir13_val = _distance_sensor[13]->getValue();
        ir14_val = _distance_sensor[14]->getValue();
        ir15_val = _distance_sensor[15]->getValue();

        cout << "ds0: " << ir0_val <<endl;
        cout <<" ds1:" << ir1_val << endl;
        cout << "ds2: " << ir2_val <<endl;
        cout <<" ds3:" << ir3_val << endl;
        cout << "ds4: " << ir4_val <<endl;
        cout <<" ds5:" << ir5_val << endl;
        cout << "ds6: " << ir6_val <<endl;
        cout <<" ds7:" << ir7_val << endl;
        cout << "ds8: " << ir8_val <<endl;
        cout <<" ds9:" << ir9_val << endl;
        cout << "ds10: " << ir10_val <<endl;
        cout <<" ds11:" << ir11_val << endl;
        cout << "ds12: " << ir12_val <<endl;
        cout <<" ds13:" << ir13_val << endl;
        cout << "ds14: " << ir14_val <<endl;
        cout <<" ds15:" << ir15_val << endl;

        // Control logic of the robot (FRONTAL ZONE)
        if ((ir15_val > DISTANCE_LIMIT)||(ir14_val > DISTANCE_LIMIT)||(ir13_val > DISTANCE_LIMIT)) {
            _mode = OBSTACLE_AVOID1;
            cout << "MOVING BACK FACING LEFT." << endl;
        }
        else {
            _mode = FORWARD;
            cout << "MOVING FORWARD." << endl;
        }

        /**if((ir12_val > ir13_val)){
            _mode = TURN_LEFT;
            cout << "TURN LEFT." << endl;
        }*/

        if ((ir0_val > DISTANCE_LIMIT)||(ir1_val > DISTANCE_LIMIT)||(ir2_val > DISTANCE_LIMIT)) {
            _mode = OBSTACLE_AVOID2;
            cout << "MOVING BACK FACING RIGHT." << endl;
        }

        if((ir3_val > DISTANCE_LIMIT+200) /*&& (ir4_val > DISTANCE_LIMIT)*/){
            _mode = OBSTACLE_AVOID2;
            cout << "MOVING BACK" << endl;

        }

         // Control logic of the robot (BACK ZONE)

        if ((ir5_val > DISTANCE_LIMIT)) {
            _mode = TURN_RIGHT;
            cout << "TURN RIGHT" << endl;
        }

        if((ir12_val > DISTANCE_LIMIT+200) /*&& (ir11_val> DISTANCE_LIMIT)*/){
            _mode = OBSTACLE_AVOID1;
            cout << "MOVING BACK." << endl;
        }

        if ((ir10_val > DISTANCE_LIMIT)) {
            _mode = TURN_LEFT;
            cout << "TURN LEFT" << endl;
        }

        if ((ir6_val > DISTANCE_LIMIT)||(ir7_val > DISTANCE_LIMIT)||(ir8_val > DISTANCE_LIMIT)||(ir9_val > DISTANCE_LIMIT)) {
            _mode = FORWARD;
            cout << "FORWARD." << endl;
        }


        // Send actuators commands according to the mode
        switch (_mode){
            case STOP:
                _left_speed = 0;
                _right_speed = 0;
                break;
            case FORWARD:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
                break;
            case TURN_LEFT:
                _left_speed = MAX_SPEED/3;
                _right_speed = MAX_SPEED ;
                break;
            case TURN_LEFT_LOW:
                _left_speed = MAX_SPEED-2;
                _right_speed = MAX_SPEED ;
                break;
            case TURN_RIGHT:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED/3 ;
                break;
            case TURN_RIGHT_LOW:
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED-2;
                break;
            case OBSTACLE_AVOID1:       //  MOVING BACK FACING LEFT
                _left_speed = -MAX_SPEED / 3.0;
                _right_speed = -MAX_SPEED / 20.0;
                break;
            case OBSTACLE_AVOID2:       //  MOVING BACK FACING RIGHT
                _left_speed = -MAX_SPEED / 20.0;
                _right_speed = -MAX_SPEED / 3.0;
                break;
            default:
                break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

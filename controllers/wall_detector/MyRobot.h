/**
 * @file    MyRobot.h
 * @brief   Header file practice 5 (wall_detector) wherein all methods and attributes are defined
 *
 * @author  Luis A. Martín <100278361@alumnos.uc3m.es>
 * @date    2014-11
 */

#include <iostream>

#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;


/**
 * @def  	MAX_SPEED
 * @brief 	maximum possible speed
 * 
 * @enum	Mode
 * @brief	enumeration of modes of the robot
 */
#define MAX_SPEED  65

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        Camera * _forward_camera;
	 enum Mode {
 
            FORWARD,
            OBSTACLE_AVOID1    
        };

        Mode _mode;

        double _left_speed, _right_speed;

    public:
        /**
         * @brief Empty constructor of the class.
         */
        MyRobot();

        /**
         * @brief Destructor of the class.
         */
        ~MyRobot();

         /**
         * @brief Function for initializing and running the Robot.
	 * @param
	 * @return
         */
        void run();
};

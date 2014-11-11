/**
 * @file    MyRobot.h
 * @brief   .h file of the pryect obstacle_avoidnace.
 *
 * @author  Luis A. Martín <100278361@alumnos.uc3m.es>
 * @date    2014-11
 */

#include <iostream>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

/**
 * @def  	NUM_DISTANCE_SENSOR
 * @brief 	total number of distance sensors
 *
 * @def  	DISTANCE_LIMIT
 * @brief 	maximum approach distance the robot
 * 
 * @def  	MAX_SPEED
 * @brief 	maximum possible speed
 */

#define NUM_DISTANCE_SENSOR 16
#define DISTANCE_LIMIT      100
#define MAX_SPEED           65


/**
 * @enum	Mode
 * @brief	enumeration of modes of the robot
 */

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];
        double _left_speed, _right_speed;

        enum Mode {
            STOP,
            FORWARD,
            TURN_LEFT,
            TURN_RIGHT,
	    TURN_LEFT_LOW,
	    TURN_RIGHT_LOW,
            OBSTACLE_AVOID1,
	    OBSTACLE_AVOID2
	    
        };

        Mode _mode;

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

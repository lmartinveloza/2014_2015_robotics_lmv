/**
 * @file    MyRobot.h
 * @brief   header file practice 4 (obstacle_odometry)
 *
 * @author  Luis A. Martín <100278361@alumnos.uc3m.es>
 * @date    2014-11
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace webots;
using namespace std;

/**
 * @def  	MAX_SPEED
 * @brief 	maximum possible speed
 */

#define MAX_SPEED  80

class MyRobot : public DifferentialWheels {
    private:

        double _left_encoder, _right_encoder;

        double _left_speed, _right_speed;

        double _distance, desired_distance, desired_angle;

        Compass* _my_compass;

        int _time_step, _status;

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
         * @brief Function for initializing and running the robot.
	 * @param
	 * @return
         */
        void run();
        
        /**
          * @brief Function for converting bearing vector from compass to angle (in degrees).
	  * @param const double* in_vector
	  * @return double variable (in degrees)
          */
        double convert_bearing_to_degrees(const double* in_vector);
};

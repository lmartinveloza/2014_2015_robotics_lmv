#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   Header file practice 5 (lines_detector) wherein all methods and attributes are defined
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
 */
#define MAX_SPEED     100

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;
        
        // wheel speeds
        double _left_speed, _right_speed;

        // camera sensor
        Camera *_spherical_camera;
 
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
        
   ;
};

#endif


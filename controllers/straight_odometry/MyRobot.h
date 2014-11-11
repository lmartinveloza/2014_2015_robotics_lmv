/**
 * @file    MyRobot.h
 * @brief   .h file for practice 4 (straight_odometry)
 *
 * @author  Luis A. Martín <100278361@alumnos.uc3m.es>
 * @date    2014-11
 */

#include <iostream>
#include <webots/DifferentialWheels.hpp>

using namespace webots;
using namespace std;


class MyRobot : public DifferentialWheels {
    private:

        double _left_encoder, _right_encoder;

        double _left_speed, _right_speed;

        double _distance, desired_distance;

        int _time_step;

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
};

/**
 * @file    MyRobot.h
 * @brief   Defining the class, private attributes and public methods that will be later implemented in the cpp file.
 *
 * @author  Pilar Molina Delgado & Luis A. Martín Veloza <100073815@alumnos.uc3m.es> & <100278361@alumnos.uc3m.es>
 * @date    2014-12
 */

#include <iostream>
#include <cmath>
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
 *
 * @var     _distance
 * @brief   estimated distance traveled by odometry
 *
 * @var     _count1
 * @brief   counter for turning on itself for the first person
 *
 * @var     _count2
 * @brief   counter for turning on itself for the second person
 *
 * @var     _persons_located
 * @brief   number of persons rescued
 *
 * @var     _door1
 * @brief   flag used to come only once a bulce saving the first person located
 *
 * @var     _door2
 * @brief   flag used to come only once a bulce saving the second person located
 *
 * @var     _signal
 * @brief   flag signal that activates when the target is very close
 *
 * @var     _time1
 * @brief   variable waiting time of the robot (in the first person)
 *
 * @var     _time2
 * @brief   variable waiting time of the robot (in the second person)
 *
 * @var     _time_signal
 * @brief   serves as a flag variable to see if has waited two seconds to start spinning on itself
 *
 * @var     _yellow_line
 * @brief   yellow line detection
 *
 */

class MyRobot : public DifferentialWheels {

private:

    double DESIRED_ANGLE;
    double _left_speed, _right_speed;
    double _left_speed, _right_speed;
    double _left_encoder, _right_encoder;
    double _distance;
    int _time_step;
    // int _prioridad;
    int _count1;
    int _count2;
    int _persons_located;
    int _door1;
    int _door2;
    int _signal;
    int _time1;
    int _time2;
    int _time_signal;
    int _yellow_line;
    int _counter_for_enabling_fcamera;
    int _count_mode;

    DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];

    Compass * _my_compass;
    Camera * _forward_camera;
    Camera * _spherical_camera;


    enum Mode
    {
        STOP,
        BACK,
        FORWARD,
        TURN,
        STRONG_TURN,
        OBSTACLE_AVOID1,
        OBSTACLE_AVOID2,
        SIGNALING_PERSON,
        BACK_HOME
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

    /**
      * @brief Function to converting bearing vector from compass to angle (in degrees).
      * @param double vector
      * @return angle in degrees (double)
      */
    double convert_bearing_to_degrees(const double* in_vector);

    /**
      * @brief Directly function to enable the distance sensors
      * @param
      * @return
      */
    void enable_distance_sensors();

    /**
      * @brief Function for the width of the captured image
      * @param
      * @return width of image (int)
      */
    int get_width();

    /**
      * @brief Function for the height of the captured image
      * @param
      * @return height of image (int)
      */
    int get_height();

    /**
      * @brief Function that always serves to turn right in function of the side readings
      * @param reading side values (sensor 3 and 4), angle of the robot (compass_angel), modification of the angle for rotation (angle_sought)
      * @return
      */
    void detection_and_turn_right(double ir3_val,double ir4_val,double compass_angle,double angle_sought);

    //REVISAR ESTO, LOS SENSORES SON LOS MISMOS QUE LA FUNCION DE ARRIBA!!
    /**
      * @brief Function that always serves to turn left in function of the side readings
      * @param reading side values (sensor 3 and 4), angle of the robot (compass_angel), modification of the angle for rotation (angle_sought)
      * @return
      */
    void detection_and_turn_left(double ir11_val,double ir12_val,double compass_angle,double angle_sought);

    /**
      * @brief Function applied to the central sensors , which acts according to the readings of the side sensors on both sides and back
      * @param reading laterals and rear sensors (sensor 11,12,3,4 and 7,8), angle of the robot (compass_angel), modification of the angle for rotation in right (angle_sought_right), modification of the angle for rotation in left (angle_sought_left)
      * @return
      */
    void obstacles_to_the_sides_look(double ir12_val,double ir11_val,double ir3_val,double ir4_val,double ir7_val,double ir8_val,double compass_angle,double angle_sought_right,double angle_sought_left);

};

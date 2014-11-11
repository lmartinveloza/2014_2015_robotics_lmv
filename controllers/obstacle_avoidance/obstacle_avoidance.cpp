/**
 * @file    obstacle_avoidance.cpp
 * @brief   file that contains the main menu of execution
 *
 * @author  Luis A. Martín <100278361@alumnos.uc3m.es>
 * @date    2014-07
 */

#include "MyRobot.h"

// This is the main program of the controller.
// It creates an instance of your Robot subclass, launches its
// function and destroys it at the end of the execution.

/**
 * @brief Main program.
 */
int main(int argc, char **argv)
{
    //Creating a dynamic variable
    MyRobot* my_robot = new MyRobot();

    //calling the function to start the simulation
    my_robot->run();

    //delete of the dinamic variable
    delete my_robot;

    return 0;
}

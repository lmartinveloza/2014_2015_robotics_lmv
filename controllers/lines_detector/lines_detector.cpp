/**
 * @file    lines_detector.cpp
 * @brief   File that contains the main menu of execution of practice 5.
 *
 * @author  Luis A. Martín <100278361@alumnos.uc3m.es>
 * @date    2014-11
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

    //Calling the function to start the simulation
    my_robot->run();

    //Delete of the dinamic variable
    delete my_robot;

    return 0;
}

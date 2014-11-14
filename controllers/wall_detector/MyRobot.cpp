/**
 * @file    MyRobot.cpp
 * @brief   Description of control code for the detection of walls.
 *
 * @author  Luis A. Martín <100278361@alumnos.uc3m.es>
 * @date    2014-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    //Initial operation mode
    _mode = FORWARD;

    //Enabling front camera
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);

}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    //Disabling front camera
    _forward_camera->disable();

}

//////////////////////////////////////////////

void MyRobot::run()
{
    //Definition and initialization of variables used
    int sum = 0;
    unsigned char green = 0, red = 0, blue = 0;
    double percentage_white = 0.0;

    // Get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;


    while (step(_time_step) != -1) {
        sum = 0;

        // Get current image from forward camera
        const unsigned char *image_f = _forward_camera->getImage();

        // Count number of pixels that are white
        for (int x = 0; x < image_width_f; x++) {
            for (int y = 0; y < image_height_f; y++) {
                //Obtaining different RGB values
                green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
                red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
                blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

                //Logical reading to find out if there is wall in front
                if ((green >= 190) && (red >= 190) && (blue >= 190)) {
                    sum = sum + 1;
                }

            }
        }

        //Display on screen the percentage of target detected by the front camera
        percentage_white = (sum / (float) (image_width_f * image_height_f)) * 100;
        cout << "Percentage of white in forward camera image: " << percentage_white << endl;

        // Control logic of the robot
        if ((percentage_white >=90)) {
            _mode = OBSTACLE_AVOID1;
            cout << "NEAREST WALL!! MOVING BACK & FACING LEFT." << endl;
        }
        else {
            _mode = FORWARD;
            cout << "MOVING FORWARD, WITHOUT BARRIERS." << endl;
        }


        // Send actuators commands according to the mode
        switch (_mode){

        case FORWARD:
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED;
            break;
        case OBSTACLE_AVOID1:       //  MOVING BACK & FACING LEFT
            _left_speed = -MAX_SPEED / 3.0;
            _right_speed = -MAX_SPEED / 20.0;
            break;

        default:
            break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

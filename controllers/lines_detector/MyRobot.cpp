/**
 * @file    MyRobot.cpp
 * @brief   Controller for controlling the cameras of the robot.
 *
 * @author  Luis A. MArtín <100278361@alumnos.uc3m.es>
 * @date    2014-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    //Definition and initialization of variables
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;


    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    //Disable of spherical camera
    _spherical_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    //Set to 0 variables
    int sum = 0;
    unsigned char green = 0, red = 0, blue = 0;
    double percentage_yellow = 0.0;


    //Get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;

    while (step(_time_step) != -1) {
        sum = 0;



        //Get current image from spherical camera
        const unsigned char *image_s = _spherical_camera->getImage();


        // count number of pixels that are yellow
        for (int x = 0; x < image_width_s; x++) {
            for (int y = 0; y < image_height_s; y++) {
                //Obtaining different RGB values
                green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
                red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
                blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

                //Logical reading to find out if there is nearest line
                if ((green > 200) && (red > 200) && (blue < 20)) {
                    sum = sum + 1;
                }
            }
        }

        //Display on screen the percentage of target detected by the spherical camera
        percentage_yellow = (sum / (float) (image_width_s * image_height_s)) * 100;
        cout << "Percentage of yellow in spherical camera image: " << percentage_yellow << endl;
        if(percentage_yellow>=1)
            cout<<"NEAREST YELLOW LINE!!!!"<<endl;

        //Turn around slowly
        _left_speed = 5;
        _right_speed = -5;

        //Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

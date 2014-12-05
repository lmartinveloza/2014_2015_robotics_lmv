/**
 * @file    MyRobot.cpp
 * @brief   Definition of all the control logic for operating the robot
 *
 * @author  Pilar Molina Delgado & Luis A. Martín Veloza <100073815@alumnos.uc3m.es> & <100278361@alumnos.uc3m.es>
 * @date    2014-12
 */




#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{

    // Initialization of all public variables of the class
    _time_step = 64;
    _left_speed = 0;
    _right_speed = 0;
    _signal=0; //señal de flag que se activa cuando el objetivo esta muy cerca
    _count1=0; //contador para dar vueltas sobre si mismo para la primera persona
    _count2=0; //contador para dar vueltas sobre si mismo para la segunda persona
    _door1=0; //sirve para entrar solo una vez al bulce de guardar la primera persona localizada
    _door2=0; //sirve para entrar solo una vez al bulce de guardar la primera segunda localizada
    _persons_located=0;
    _time1=0;
    _time2=0;
    _left_encoder=0;
    _right_encoder=0;
    _time_signal=0; //variable que sirve de flag para saber si ha esperado 2 segundos para empezar a dar vueltas sobre si mismo
    _yellow_line=0;
    _distance = 0;
    _counter_for_enabling_fcamera=0;
    _count_mode=0;
    // _prioridad=0;
    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    // Enable spherical camara
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);

    // Initial operating mode
    _mode =FORWARD;

    // Function to activate directly the distance sensors
    enable_distance_sensors();

}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // Desactivation of all distance sensors, compass, encoder and diferent cameras
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
    {
        _distance_sensor[i]->disable();
    }
    _my_compass->disable();
    _forward_camera->disable();
    _spherical_camera->disable();
    disableEncoders();



}

//////////////////////////////////////////////

void MyRobot::run()
{
    // Initialization of global varibales used
    double compass_angle;
    int limit1=170, limit2=170, time_2_seconds_1=25, time_2_seconds_2=25, line=0, time_whitout_camera=200;

    // Initialization values ​​of all sensor readings
    double ir0_val = 0.0, ir1_val = 0.0, ir2_val = 0.0, ir3_val = 0.0, ir4_val = 0.0;
    double ir5_val = 0.0, ir6_val = 0.0, ir7_val = 0.0, ir8_val = 0.0, ir9_val = 0.0;
    double ir10_val = 0.0, ir11_val = 0.0, ir12_val = 0.0, ir13_val = 0.0, ir14_val = 0.0, ir15_val=0.0;

    // General-Logic when the simulation begins
    while (step(_time_step) != -1)
    {

        // Read the sensors of distance
        ir0_val = _distance_sensor[0]->getValue();
        ir1_val = _distance_sensor[1]->getValue();
        ir2_val = _distance_sensor[2]->getValue();
        ir3_val = _distance_sensor[3]->getValue();
        ir4_val = _distance_sensor[4]->getValue();
        ir5_val = _distance_sensor[5]->getValue();
        ir6_val = _distance_sensor[6]->getValue();
        ir7_val = _distance_sensor[7]->getValue();
        ir8_val = _distance_sensor[8]->getValue();
        ir9_val = _distance_sensor[9]->getValue();
        ir10_val = _distance_sensor[10]->getValue();
        ir11_val = _distance_sensor[11]->getValue();
        ir12_val = _distance_sensor[12]->getValue();
        ir13_val = _distance_sensor[13]->getValue();
        ir14_val = _distance_sensor[14]->getValue();
        ir15_val = _distance_sensor[15]->getValue();


        // Screen printing for all values ​​read by the sensors away
        cout << "ds0: " << ir0_val <<endl;
        cout <<" ds1:" << ir1_val << endl;
        cout << "ds2: " << ir2_val <<endl;
        cout <<" ds3:" << ir3_val << endl;
        cout << "ds4: " << ir4_val <<endl;
        cout <<" ds5:" << ir5_val << endl;
        cout << "ds6: " << ir6_val <<endl;
        cout <<" ds7:" << ir7_val << endl;
        cout << "ds8: " << ir8_val <<endl;
        cout <<" ds9:" << ir9_val << endl;
        cout << "ds10: " << ir10_val <<endl;
        cout <<" ds11:" << ir11_val << endl;
        cout << "ds12: " << ir12_val <<endl;
        cout <<" ds13:" << ir13_val << endl;
        cout << "ds14: " << ir14_val <<endl;
        cout <<" ds15:" << ir15_val << endl;

        // Read the compass
        const double *compass_val = _my_compass->getValues();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;



        //////////////////////////////////////////////
        // Forward camera control logic ((detection of yellow line)


        if (line==1)
        {

            // Initialization of local varibales used
            int sum = 0;
            unsigned char green_f = 0, red_f = 0, blue_f = 0;
            double percentage_yellow_f= 0.0;


            // Get size of images for forward camera
            // Function to directly obtain the width of the captured image
            int image_width_f = get_width();
            // Function to directly obtain the height of the captured image
            int image_height_f =get_height();


            // Get current image from forward camera
            const unsigned char *image_f = _forward_camera->getImage();

            // Count number of pixels that are yellow in the captured image
            for (int x = 0; x < image_width_f; x++)
            {
                for (int y = 0; y < image_height_f; y++)
                {
                    // Obtaining different RGB values
                    green_f = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
                    red_f = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
                    blue_f = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

                    // Logical reading to find out if there is a yellow line in front
                    if ((blue_f<=20) && ((red_f/green_f)>=0.8) && ((red_f/green_f)<=1.1) && (red_f>=20) && (green_f>=20))
                    {
                        sum = sum + 1;
                    }
                }
            }

            // Save percentage of target detected by the front camera
            percentage_yellow_f = (sum / (float) (image_width_f * image_height_f)) * 100;
            sum=0;

            // Logic of vision yellow line
            if (percentage_yellow_f>0.80 && percentage_yellow_f<1)
            {
                cout<<"Yellow line displays"<<endl;
                _yellow_line=1;
            }

        }

        //////////////////////////////////////////////

        //////////////////////////////////////////////
        // Spherical camera control logic (detection of persons)

        // Definition and initialization of variables used
        int  pix1=0,pix2=0,pix3=0,pix4=0;
        unsigned char green = 0, red = 0, blue = 0;

        double percentage_green_s1= 0.0;
        double percentage_green_s2= 0.0;
        double percentage_green_s3= 0.0;
        double percentage_green_s4= 0.0;

        // Get size of images for spherical camera
        int image_width_s = _spherical_camera->getWidth();
        int image_height_s = _spherical_camera->getHeight();

        // Get current image from forward camera
        const unsigned char *image_s = _spherical_camera->getImage();

        // Count number of pixels that are green

        // Setting spherical image: left side
        for (int x = 0; x < (image_width_s/4); x++)
        {
            for (int y = 0; y < image_height_s; y++)
            {
                // Values RGB
                green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
                red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
                blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

                // To detect the green RGB values
                if ((green > 75) && ((green-75)/red >= 2) && ((green-75)/blue>=2))
                {
                    // Having green pixel in that area
                    pix1 = pix1 + 1;
                }
            }
        }

        // Setting spherical image: down side
        for (int x =(image_width_s/4); x < image_width_s; x++)
        {
            for (int y = (3*(image_height_s/4)); y < image_height_s; y++)
            {
                // Values RGB
                green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
                red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
                blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

                // To detect the green RGB values
                if ((green > 75) && ((green-75)/red >= 2) && ((green-75)/blue>=2))
                {
                    // Having green pixel in that area
                    pix2 = pix2 + 1;
                }
            }
        }

        // Setting spherical image: up side
        for (int x =(image_width_s/4); x < image_width_s; x++)
        {
            for (int y = 0; y < (image_height_s/4); y++)
            {
                // Values RGB
                green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
                red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
                blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

                // To detect the green RGB values
                if ((green > 75) && ((green-75)/red >= 2) && ((green-75)/blue>=2))
                {
                    // Having green pixel in that area
                    pix3 = pix3 + 1;
                }
            }
        }

        // Setting spherical image: right side
        for (int x =(3*(image_width_s/4)); x < image_width_s; x++)
        {
            for (int y = (image_height_s/4); y < (3*(image_height_s/4)); y++)
            {
                // Values RGB
                green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
                red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
                blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

                // To detect the green RGB values
                if ((green > 75) && ((green-75)/red >= 2) && ((green-75)/blue>=2))
                {
                    // Having green pixel in that area
                    pix4 = pix4 + 1;

                }

            }
        }


        // Display on screen the percentage of target detected by the front camera
        percentage_green_s1 = (pix1 / (float) (image_width_s * image_height_s)) * 100;
        percentage_green_s2 = (pix2 / (float) (image_width_s * image_height_s)) * 100;
        percentage_green_s3 = (pix3 / (float) (image_width_s * image_height_s)) * 100;
        percentage_green_s4 = (pix4 / (float) (image_width_s * image_height_s)) * 100;

        cout << "Percentage of green in forward camera image: " << percentage_green_s1<<"Person in left"<< endl;
        cout << "Percentage of green in forward camera image: " << percentage_green_s2<<"Person in back"<< endl;
        cout << "Percentage of green in forward camera image: " << percentage_green_s3<<"Person in front"<< endl;
        cout << "Percentage of green in forward camera image: " << percentage_green_s4<<"Person in right"<< endl;



        cout<<"Persons rescued: "<<_persons_located<<endl;

        /** cout<<"LINEAS AMRILLAS LOCALIZADAS: "<<_yellow_line<<endl;*/

        //////////////////////////////////////////////

        //////////////////////////////////////////////
        // Principal control loop

        if(((ir0_val > DISTANCE_LIMIT) ||  (ir1_val > DISTANCE_LIMIT) || (ir2_val > DISTANCE_LIMIT) ||(ir13_val > DISTANCE_LIMIT) || (ir14_val > DISTANCE_LIMIT) ||(ir15_val > DISTANCE_LIMIT)) || ((pix1>0||pix2>0||pix3>0||pix4>0) && _persons_located!=2) || _yellow_line==1)
        {
            // If it detects that we reach the end of the world, habilitation of odometry
            if(_yellow_line==1)
            {
                //Habilitation of enconders
                //IMPORTANT NOTE: IT IS NECESSARY THAT "ENCODER RESOLUTION"=5 AND "ENCODEER NOISE"=0 IN THE SIMULATION
                enableEncoders(_time_step);

                // Calculation of distance estimated by odometry
                //Assigning values ​​to the corresponding variables read
                _left_encoder = getLeftEncoder();
                _right_encoder = getRightEncoder();
                _distance = _left_encoder/60.61;


                cout<< "Estimated distance travel: "<<_distance<< endl;

                // If we have traveled enough meters to cross the line
                if(_distance>=4.80)
                {
                    _mode=STOP;
                    cout<<"MISION COMPLETED"<<endl;
                }
                else
                    _mode=BACK_HOME;
            }

            // If a person is detected and have not rescued two persons
            if((pix1>0||pix2>0||pix3>0||pix4>0) && (_persons_located==0 || _persons_located==1))
            {

                // If green is detected in greater proportion in the left side (pix1)
                if (pix1>pix2 && pix1>pix3 && pix1>pix4)
                {

                    // We turned that direction
                    DESIRED_ANGLE=compass_angle-90;
                    _mode=TURN;

                    cout<<"Come to the target"<<endl;

                    // If we are going in the right direction
                    if (DESIRED_ANGLE==compass_angle)
                        _mode=FORWARD;

                    // If we have to target near (we set a flag of person found _signal)
                    if(percentage_green_s3>=8)
                    {
                        _signal=1;
                    }

                    // If we close and touch the target with sensors ( 1 meter )
                    if(_signal==1 && ((ir0_val>0 && ir0_val<DISTANCE_LIMIT) ||(ir1_val>0 && ir1_val<DISTANCE_LIMIT) ||(ir15_val>0 && ir15_val<DISTANCE_LIMIT) ||(ir14_val>0 && ir14_val<DISTANCE_LIMIT)))
                    {

                        // Disable the distance sensors
                        for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
                        {
                            _distance_sensor[i]->disable();
                        }

                        // We proceed to signal the rescue
                        _mode=SIGNALING_PERSON;
                    }
                }

                // If green is detected in greater proportion in the back side (pix2)
                if (pix2>pix1 && pix2>pix3 && pix2>pix4)
                {
                    DESIRED_ANGLE=compass_angle-180;

                    cout<<"Come to the target"<<endl;
                    _mode=TURN;

                    if (DESIRED_ANGLE==compass_angle)
                        _mode=FORWARD;

                    if(percentage_green_s3>=8)
                    {
                        _signal=1;
                    }

                    if(_signal==1 && ((ir0_val>0 && ir0_val<DISTANCE_LIMIT) ||(ir1_val>0 && ir1_val<DISTANCE_LIMIT) ||(ir15_val>0 && ir15_val<DISTANCE_LIMIT) ||(ir14_val>0 && ir14_val<DISTANCE_LIMIT)))
                    {

                        for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
                        {
                            _distance_sensor[i]->disable();
                        }

                        _mode=SIGNALING_PERSON;
                    }
                }

                // If green is detected in greater proportion in the right side (pix4)
                if (pix4>pix2 && pix4>pix3 && pix4>pix1)
                {
                    DESIRED_ANGLE=compass_angle+90;

                    cout<<"Come to the target"<<endl;
                    _mode=TURN;

                    if (DESIRED_ANGLE==compass_angle)
                        _mode=FORWARD;

                    if(percentage_green_s3>=8)
                    {
                        _signal=1;
                    }
                    if(_signal==1 && ((ir0_val>0 && ir0_val<DISTANCE_LIMIT) ||(ir1_val>0 && ir1_val<DISTANCE_LIMIT) ||(ir15_val>0 && ir15_val<DISTANCE_LIMIT) ||(ir14_val>0 && ir14_val<DISTANCE_LIMIT)))
                    {

                        for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
                        {
                            _distance_sensor[i]->disable();
                        }

                        _mode=SIGNALING_PERSON;
                    }
                }

                // If green is detected in greater proportion in the front side (pix3)
                if (pix3>pix4 && pix3>pix2 && pix3>pix1)
                {
                    DESIRED_ANGLE=compass_angle;

                    cout<<"Come to the target"<<endl;
                    _mode=TURN;

                    if (DESIRED_ANGLE==compass_angle)
                        _mode=FORWARD;

                    if(percentage_green_s3>=8)
                    {
                        _signal=1;
                    }

                    if(_signal==1 && ((ir0_val>0 && ir0_val<DISTANCE_LIMIT) ||(ir1_val>0 && ir1_val<DISTANCE_LIMIT) ||(ir15_val>0 && ir15_val<DISTANCE_LIMIT) ||(ir14_val>0 && ir14_val<DISTANCE_LIMIT)))
                    {

                        for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
                        {
                            _distance_sensor[i]->disable();
                        }

                        _mode=SIGNALING_PERSON;
                    }
                }
            }




            // If we have reached the end of the count to signal rescue the first person rescued (_count1)
            // _door1 variable used to enter only once this bulce
            if(_count1==limit1 && _door1==0)
            {
                // Increased variable for the loop input condition is no longer satisfied
                _door1++;

                _persons_located=1;

                // Resetting variables person very close ( _signal ) and waiting 2 seconds ( _time_signal )
                _signal=0;
                _time_signal=0;

                //Activation of distance sensors to continue the search for the next target
                enable_distance_sensors();
            }

            // If we have reached the end of the count to signal rescue the second person rescued (_count2)
            // _door2 variable used to enter only once this bulce
            if(_count2==limit2 && _door2==0)
            {
                // Increased variable for the loop input condition is no longer satisfied
                _door2++;

                _persons_located=2;

                _signal=0;
                _time_signal=0;

                enable_distance_sensors();

                // We finished rescue 2 people , so we do not need the spherical camera
                //Disabling spherical camara
                _spherical_camera->disable();

                // We return to the beginning of the world
                _mode=BACK_HOME;
            }



            // Logical control area of distance sensors
            if(/**_prioridad==0 && */((ir0_val > DISTANCE_LIMIT) ||  (ir1_val > DISTANCE_LIMIT) || (ir2_val > DISTANCE_LIMIT) ||(ir13_val > DISTANCE_LIMIT) || (ir14_val > DISTANCE_LIMIT) ||(ir15_val > DISTANCE_LIMIT)))
            {
                // Case detection sensors on left quadrant
                // Detection of sensor 2

                if((ir2_val > DISTANCE_LIMIT) &&  (ir1_val < DISTANCE_LIMIT) && (ir0_val < DISTANCE_LIMIT) &&  (ir15_val < DISTANCE_LIMIT) && (ir14_val < DISTANCE_LIMIT) && (ir13_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_right(ir3_val,ir4_val,compass_angle,30);
                }

                // Detection of sensor 1
                if((ir1_val > DISTANCE_LIMIT) &&  (ir2_val < DISTANCE_LIMIT) && (ir0_val < DISTANCE_LIMIT) &&  (ir15_val < DISTANCE_LIMIT) && (ir14_val < DISTANCE_LIMIT) && (ir13_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_right(ir3_val,ir4_val,compass_angle,35.5);
                }


                // Detection of sensor 0
                if((ir0_val > DISTANCE_LIMIT) &&  (ir2_val < DISTANCE_LIMIT) && (ir1_val < DISTANCE_LIMIT) &&  (ir15_val < DISTANCE_LIMIT) && (ir14_val < DISTANCE_LIMIT) && (ir13_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_right(ir3_val,ir4_val,compass_angle,42.5);
                }

                // Detection of sensor 1 and 2
                if((ir1_val > DISTANCE_LIMIT) &&  (ir2_val > DISTANCE_LIMIT) && (ir0_val < DISTANCE_LIMIT) &&  (ir15_val < DISTANCE_LIMIT) && (ir14_val < DISTANCE_LIMIT) && (ir13_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_right(ir3_val,ir4_val,compass_angle,35.5);
                }


                // Detection of sensor 0 and 1
                if((ir0_val > DISTANCE_LIMIT) &&  (ir1_val > DISTANCE_LIMIT) && (ir15_val < DISTANCE_LIMIT) &&  (ir14_val < DISTANCE_LIMIT) && (ir13_val < DISTANCE_LIMIT) && (ir2_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_right(ir3_val,ir4_val,compass_angle,35.5);
                }

                // Detection of sensor 0, 1 and 2
                if((ir1_val > DISTANCE_LIMIT) &&  (ir2_val > DISTANCE_LIMIT) && (ir0_val > DISTANCE_LIMIT) && (ir15_val < DISTANCE_LIMIT) &&  (ir14_val < DISTANCE_LIMIT) && (ir13_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_right(ir3_val,ir4_val,compass_angle,40);
                }


                // Case detection sensors on front
                // Detection of sensor 0 and 15
                if((ir0_val > DISTANCE_LIMIT) &&  (ir15_val > DISTANCE_LIMIT)&& (ir14_val < DISTANCE_LIMIT) &&  (ir13_val < DISTANCE_LIMIT) && (ir1_val < DISTANCE_LIMIT) && (ir2_val < DISTANCE_LIMIT))
                    //aumento distancia porque si no se acera mucho
                {
                    obstacles_to_the_sides_look(ir12_val,ir11_val,ir3_val,ir4_val,ir7_val,ir8_val,compass_angle,45,45);
                }

                // Detection of sensor 0, 14 and 15
                if((ir0_val > DISTANCE_LIMIT) &&  (ir15_val > DISTANCE_LIMIT) && (ir14_val > DISTANCE_LIMIT) &&  (ir13_val < DISTANCE_LIMIT) && (ir1_val < DISTANCE_LIMIT) && (ir2_val < DISTANCE_LIMIT))
                {
                    obstacles_to_the_sides_look(ir12_val,ir11_val,ir3_val,ir4_val,ir7_val,ir8_val,compass_angle,45,45);
                }

                // Detection of sensor 0, 1 and 15
                if((ir0_val > DISTANCE_LIMIT) &&  (ir15_val > DISTANCE_LIMIT) && (ir1_val > DISTANCE_LIMIT) &&  (ir13_val < DISTANCE_LIMIT) && (ir14_val < DISTANCE_LIMIT) && (ir2_val < DISTANCE_LIMIT))
                {
                    obstacles_to_the_sides_look(ir12_val,ir11_val,ir3_val,ir4_val,ir7_val,ir8_val,compass_angle,45,45);
                }

                // Detection of sensor 0, 1, 2, 13, 14 and 15
                if((ir0_val > DISTANCE_LIMIT) &&  (ir15_val > DISTANCE_LIMIT) && (ir1_val > DISTANCE_LIMIT) &&  (ir13_val > DISTANCE_LIMIT) && (ir14_val > DISTANCE_LIMIT) && (ir2_val > DISTANCE_LIMIT))
                {
                    obstacles_to_the_sides_look(ir12_val,ir11_val,ir3_val,ir4_val,ir7_val,ir8_val,compass_angle,45,45);
                }


                // Case detection sensors on right quadrant
                // Detection of sensor 13
                if((ir13_val > DISTANCE_LIMIT) &&  (ir14_val < DISTANCE_LIMIT) && (ir15_val < DISTANCE_LIMIT) &&  (ir0_val < DISTANCE_LIMIT) && (ir1_val < DISTANCE_LIMIT) && (ir2_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_left(ir11_val,ir12_val,compass_angle,45);
                }

                // Detection of sensor 14
                if((ir14_val > DISTANCE_LIMIT) &&  (ir13_val < DISTANCE_LIMIT) && (ir15_val < DISTANCE_LIMIT) &&  (ir0_val < DISTANCE_LIMIT) && (ir1_val < DISTANCE_LIMIT) && (ir2_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_left(ir11_val,ir12_val,compass_angle,35.5);
                }

                // Detection of sensor 15
                if((ir15_val > DISTANCE_LIMIT) &&  (ir14_val < DISTANCE_LIMIT) && (ir13_val < DISTANCE_LIMIT) &&  (ir0_val < DISTANCE_LIMIT) && (ir1_val < DISTANCE_LIMIT) && (ir2_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_left(ir11_val,ir12_val,compass_angle,42.5);
                }

                // Detection of sensor 13 and 14
                if((ir13_val > DISTANCE_LIMIT) &&  (ir14_val > DISTANCE_LIMIT)&& (ir15_val < DISTANCE_LIMIT) &&  (ir0_val < DISTANCE_LIMIT) && (ir1_val < DISTANCE_LIMIT) && (ir2_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_left(ir11_val,ir12_val,compass_angle,25.5);
                }

                // Detection of sensor 14 and 15
                if((ir15_val > DISTANCE_LIMIT) &&  (ir14_val > DISTANCE_LIMIT) && (ir12_val < DISTANCE_LIMIT) &&  (ir0_val < DISTANCE_LIMIT) && (ir1_val < DISTANCE_LIMIT) && (ir2_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_left(ir11_val,ir12_val,compass_angle,35.5);
                }

                // Detection of sensor 13, 14 and 15
                if((ir15_val > DISTANCE_LIMIT) &&  (ir14_val > DISTANCE_LIMIT) && (ir13_val > DISTANCE_LIMIT)&& (ir0_val < DISTANCE_LIMIT) &&  (ir1_val < DISTANCE_LIMIT) && (ir2_val < DISTANCE_LIMIT))
                {
                    detection_and_turn_left(ir11_val,ir12_val,compass_angle,40);
                }

            }
        }
        else
        {
            if(_persons_located==0)
                _mode= FORWARD;

            if(_persons_located==2)
                _mode= BACK_HOME;
        }

        //////////////////////////////////////////////

        //////////////////////////////////////////////
        // Zone definition of robot modes

        // Send actuators commands according to the mode
        switch (_mode){

        case STOP:
            cout<<"MODE ACTIVATED: STOP"<<endl;
            _left_speed = 0;
            _right_speed = 0;
            break;

        case BACK:
            cout<<"MODE ACTIVATED: BACK"<<endl;
            _left_speed = -MAX_SPEED;
            _right_speed = -MAX_SPEED;
            break;

        case FORWARD:
            cout<<"MODE ACTIVATED: FORWARD"<<endl;
            DESIRED_ANGLE=45;

            if(compass_angle < (DESIRED_ANGLE - 2))
            {
                // Turn right
                cout<<"FORWARD MODE AND BANG TURN RIGHT"<<endl;
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED - 30;
            }

            if(compass_angle > (DESIRED_ANGLE - 2))
            {
                // Turn left
                cout<<"FORWARD MODE AND BANG TURN LEFT"<<endl;
                _left_speed = MAX_SPEED - 30;
                _right_speed = MAX_SPEED;
            }
            break;

        case TURN:
            if (compass_angle < (DESIRED_ANGLE - 2)) {
                // Turn right
                cout<<"TURN RIGHT"<<endl;
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED - 45;
            }

            if(compass_angle > (DESIRED_ANGLE - 2))
            {
                // Turn left
                cout<<"TURN LEFT"<<endl;
                _left_speed = MAX_SPEED - 45;
                _right_speed = MAX_SPEED;
            }
            else
            {
                _mode=FORWARD;
            }
            break;

        case STRONG_TURN:
            if (compass_angle < (DESIRED_ANGLE - 2))
            {
                // Turn right
                cout<<"TURN RIGHT"<<endl;
                _left_speed = MAX_SPEED;
                _right_speed = 0;
            }

            if(compass_angle > (DESIRED_ANGLE - 2))
            {
                // Turn left
                cout<<"TURN LEFT"<<endl;
                _left_speed = 0;
                _right_speed = MAX_SPEED;
            }
            else
            {
                _mode=FORWARD;
            }
            break;


        case OBSTACLE_AVOID1:
            //  Moving back and facing left
            cout<<"MODE ACTIVATED: OSBTACLE AVOID 1 (Moving back and facing left)"<<endl;
            _left_speed = -MAX_SPEED / 5;
            _right_speed = -MAX_SPEED / 30.0;
            break;

        case OBSTACLE_AVOID2:
            //  Moving back and facing right
            cout<<"MODE ACTIVATED: OBSTACLE AVOID 2 (Moving back and facing right)"<<endl;
            _left_speed = -MAX_SPEED / 30.0;
            _right_speed = -MAX_SPEED / 5;
            break;

        case SIGNALING_PERSON:
            // If the time to waiting (in the first person) is less than the time set in the variable (tiempo_2_segundos_1)
            if(_time1<time_2_seconds_1)
            {
                cout<<"Waiting 2 seconds"<<endl;
                _time1++;
                _left_speed = 0;
                _right_speed = 0;
            }

            // If we have already complied with the time of turning on itself , activates the flag (_time_signal)
            if(_time1==time_2_seconds_1)
                _time_signal=1;

            // If the time to waiting (in the second person) is less than the time set in the variable (tiempo_2_segundos_2)
            if(_time2<time_2_seconds_2)
            {
                cout<<"Waiting 2 seconds"<<endl;
                _time2++;
                _left_speed = 0;
                _right_speed = 0;
            }

            if(_time2==time_2_seconds_2)
                _time_signal=1;

            // If the flag (_time_signal) is enabled
            if(_time_signal==1)
            {

                if(_persons_located==0)
                    _count1++;

                if(_persons_located==1)
                    _count2++;

                cout<<"MODE ACTIVATED: SIGNALING PERSON"<<endl;

                // The robot turns on itself for signaling to the person
                DESIRED_ANGLE=compass_angle+1;
                if (compass_angle <DESIRED_ANGLE )
                {
                    // Turn on itself
                    _left_speed = -MAX_SPEED/4;
                    _right_speed = MAX_SPEED/4;
                }
            }
            break;

        case BACK_HOME:
            if(_counter_for_enabling_fcamera<time_whitout_camera)
                _counter_for_enabling_fcamera++;
            if(_counter_for_enabling_fcamera==time_whitout_camera)
            {

                //Enabling front camera
                // It's necessary to see where they stop later
                _forward_camera = getCamera("camera_f");
                _forward_camera->enable(_time_step);

                // Enable flag (line) to enter logical loop control of forward camera
                line=1;
            }

            cout<<"MODE ACTIVATED: BACK HOME"<<endl;

            DESIRED_ANGLE=-135;

            if(compass_angle>=45 || (compass_angle>=-180 && compass_angle<-135))
            {
                // Turn right
                cout<<"BACK HOME MODE AND BANG TURN RIGHT"<<endl;
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED - 20;
            }

            if((compass_angle>-135 && compass_angle<45))
            {
                // Turn left
                cout<<"BACK HOME MODE AND BANG TURN LEFT"<<endl;
                _left_speed = MAX_SPEED - 20;
                _right_speed = MAX_SPEED;
            }
            break;

        default:
            break;
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}


//////////////////////////////////////////////

//Zone definition of created functions

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////

void MyRobot::enable_distance_sensors()
{

    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds3");
    _distance_sensor[3]->enable(_time_step);
    _distance_sensor[4] = getDistanceSensor("ds4");
    _distance_sensor[4]->enable(_time_step);
    _distance_sensor[5] = getDistanceSensor("ds5");
    _distance_sensor[5]->enable(_time_step);
    _distance_sensor[6] = getDistanceSensor("ds6");
    _distance_sensor[6]->enable(_time_step);
    _distance_sensor[7] = getDistanceSensor("ds7");
    _distance_sensor[7]->enable(_time_step);
    _distance_sensor[8] = getDistanceSensor("ds8");
    _distance_sensor[8]->enable(_time_step);
    _distance_sensor[9] = getDistanceSensor("ds9");
    _distance_sensor[9]->enable(_time_step);
    _distance_sensor[10] = getDistanceSensor("ds10");
    _distance_sensor[10]->enable(_time_step);
    _distance_sensor[11] = getDistanceSensor("ds11");
    _distance_sensor[11]->enable(_time_step);
    _distance_sensor[12] = getDistanceSensor("ds12");
    _distance_sensor[12]->enable(_time_step);
    _distance_sensor[13] = getDistanceSensor("ds13");
    _distance_sensor[13]->enable(_time_step);
    _distance_sensor[14] = getDistanceSensor("ds14");
    _distance_sensor[14]->enable(_time_step);
    _distance_sensor[15] = getDistanceSensor("ds15");
    _distance_sensor[15]->enable(_time_step);
}

//////////////////////////////////////////////

int MyRobot::get_width()
{
    return  _forward_camera->getWidth();
}

//////////////////////////////////////////////

int MyRobot::get_height()
{
    return  _forward_camera->getHeight();
}
//////////////////////////////////////////////

void MyRobot::detection_and_turn_right(double ir3_val, double ir4_val,double compass_angle, double angle_sought)
{
    if(ir3_val> DISTANCE_LIMIT ||ir4_val> DISTANCE_LIMIT )
    {
        // Turn stronger for having left wall and front
        DESIRED_ANGLE=compass_angle+angle_sought;
        _mode=STRONG_TURN;
    }
    else
    {
        // Turn to the right (+)
        DESIRED_ANGLE=compass_angle+angle_sought;
        _mode=TURN;
    }
}
//////////////////////////////////////////////

void MyRobot::detection_and_turn_left(double ir11_val, double ir12_val, double compass_angle, double angle_sought)
{
    if(ir11_val> DISTANCE_LIMIT ||ir12_val> DISTANCE_LIMIT )
    {
        // Turn stronger for having left wall and front
        DESIRED_ANGLE=compass_angle-angle_sought;
        _mode=STRONG_TURN;
    }
    else
    {
        // Turn to the right(-)
        DESIRED_ANGLE=compass_angle-angle_sought;
        _mode=TURN;
    }
}
//////////////////////////////////////////////

void MyRobot::obstacles_to_the_sides_look(double ir12_val, double ir11_val, double ir3_val, double ir4_val, double ir7_val, double ir8_val,double compass_angle,double angle_sought_right,double angle_sought_left)
{

    if((ir12_val > DISTANCE_LIMIT) ||(ir11_val > DISTANCE_LIMIT) ||(ir3_val > DISTANCE_LIMIT) ||(ir4_val > DISTANCE_LIMIT) ||(ir7_val > DISTANCE_LIMIT) ||(ir8_val > DISTANCE_LIMIT))
    {
        // See if there obstacle to the right
        if((ir12_val > DISTANCE_LIMIT) || (ir11_val > DISTANCE_LIMIT))
        {
            // See if there obstacle to the back
            if((ir7_val > DISTANCE_LIMIT) || (ir8_val > DISTANCE_LIMIT))
            {
                // Turn left orientation (-)
                DESIRED_ANGLE=compass_angle-angle_sought_left;
                _mode=STRONG_TURN;
            }
            else
            {
                _mode=OBSTACLE_AVOID1;
            }
        }

        //See if there obstacle to the left
        if((ir3_val > DISTANCE_LIMIT) || (ir4_val > DISTANCE_LIMIT))
        {
            // See if there obstacle to the back
            if((ir7_val > DISTANCE_LIMIT) || (ir8_val > DISTANCE_LIMIT))
            {
                // Turn right orientation (+)
                DESIRED_ANGLE=compass_angle+angle_sought_right;
                _mode=STRONG_TURN;
            }

            else
            {
                _mode=OBSTACLE_AVOID2;
            }

            // Look if obstacles on both sides
            if(((ir12_val > DISTANCE_LIMIT) || (ir11_val > DISTANCE_LIMIT)) && ((ir3_val > DISTANCE_LIMIT) || (ir4_val > DISTANCE_LIMIT)))
            {
                // Reversing
                _mode=BACK;
            }

            // See if there obstacle to the back
            if((ir7_val > DISTANCE_LIMIT) || (ir8_val > DISTANCE_LIMIT))
            {

                //Turn left orientation (-)
                DESIRED_ANGLE=compass_angle-angle_sought_left;
                _mode=STRONG_TURN;
            }
        }
    }
    else
    {
        if(_count_mode==0)
        {
            _mode=OBSTACLE_AVOID1;
            _count_mode=1;
        }
        if(_count_mode==1)
        {
            _mode=OBSTACLE_AVOID2;
            _count_mode=0;
        }
    }
}
//////////////////////////////////////////////

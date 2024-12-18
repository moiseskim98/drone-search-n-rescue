/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Simplistic drone control:
 * - Stabilize the robot using the embedded sensors.
 * - Use PID technique to stabilize the drone roll/pitch/yaw.
 * - Use a cubic function applied on the vertical difference to stabilize the robot vertically.
 * - Stabilize the camera.
 * - Control the robot using the computer keyboard.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/camera_recognition_object.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define ID 187
#define EPSILON 0.1
#define EPSILON_DROP 2.2


struct obj_detected_t {
  double x;
  double y;
  double theta_x;
  double theta_y;
  char name;
  double id;
};

// yaw PID
double kpOx = 1.50;
double kiOx = 0.0010; 
double kdOx = 0.1000;
double EOx = 0;
double eO_1x = 0;

// pitch PID
double kpOy = 5.00;
double kiOy = 0.010; 
double kdOy = 0.0001;
double EOy = 0;
double eO_1y = 0;

// exponential controller
double alfa = 0.2;
double v0 = 2.5; // max speed

int state;


int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Get and enable devices.
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  wb_camera_recognition_enable(camera, timestep);
  wb_camera_recognition_enable_segmentation(camera);
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
  WbDeviceTag latch_motor = wb_robot_get_device("latch");
  // WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");  // Not used in this example.

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  int m;
  for (m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  // Display the welcome message.
  printf("Start the drone...\n");

  // Wait one second.
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  // Display manual control message.
  printf("You can control the drone with your computer keyboard:\n");
  printf("- 'up': move forward.\n");
  printf("- 'down': move backward.\n");
  printf("- 'right': turn right.\n");
  printf("- 'left': turn left.\n");
  printf("- 'shift + up': increase the target altitude.\n");
  printf("- 'shift + down': decrease the target altitude.\n");
  printf("- 'shift + right': strafe right.\n");
  printf("- 'shift + left': strafe left.\n");
  printf("- 'A': open latch.\n");
  printf("- 'S': close latch.\n");

  // Constants, empirically found.
  const double k_vertical_thrust = 70;//68.5;  // with this thrust, the drone lifts.
  //const double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  const double k_vertical_offset = 1.6;   // Vertical offset where the robot actually targets to stabilize itself.
  const double k_vertical_p = 3.0;        // P constant of the vertical PID.
  const double k_roll_p = 50.0;           // P constant of the roll PID.
  const double k_pitch_p = 30.0;          // P constant of the pitch PID.

  // Variables.
  double target_altitude = 1.0;  // The target altitude. Can be changed by the user.
  double latch_position = 0;
  const WbCameraRecognitionObject *object;
  struct obj_detected_t obj;
  obj.id = ID;
  double fx, fy;
  state = 0;

  // Main loop
  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();  // in seconds.

    // Retrieve the number of elements detected by the camera
    int num_objects = wb_camera_recognition_get_number_of_objects(camera);
    object = wb_camera_recognition_get_objects(camera);
    
    
    // Retrieve robot position using the sensors.
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double altitude = wb_gps_get_values(gps)[2];
    const double roll_velocity = wb_gyro_get_values(gyro)[0];
    const double pitch_velocity = wb_gyro_get_values(gyro)[1];

    // Blink the front LEDs alternatively with a 1 second rate.
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);
    
    

    // Transform the keyboard input to disturbances on the stabilization algorithm.
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;
    double camera_yaw_disturbance = 0.5;
    
    
    
    switch (state)
    {
      case 0:
        target_altitude = 0;
        //double epsilon = 0.1;
        
        printf("height: %f\n",altitude);
        
        double error = pow(target_altitude - altitude,2);
        
        if(error < EPSILON){state=1;}
        break;
        
        
      case 1:
        int key = wb_keyboard_get_key();
        while (key > 0) {
          switch (key) {
            case WB_KEYBOARD_UP:
              pitch_disturbance = -2.0;
              break;
            case WB_KEYBOARD_DOWN:
              pitch_disturbance = 2.0;
              break;
            case WB_KEYBOARD_RIGHT:
              yaw_disturbance = -1.3;
              break;
            case WB_KEYBOARD_LEFT:
              yaw_disturbance = 1.3;
              break;
            case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
              roll_disturbance = -1.0;
              break;
            case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
              roll_disturbance = 1.0;
              break;
            case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
              target_altitude += 0.05;
              printf("target altitude: %f [m]\n", target_altitude);
              break;
            case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
              target_altitude -= 0.05;
              printf("target altitude: %f [m]\n", target_altitude);
              break;
            case ('A'):
              latch_position = 0.5;
              break;
            case ('S'):
              latch_position = 0.0;
              break;
          }
          key = wb_keyboard_get_key();
        }
        
        /*
        for(int i = 0; i<num_objects; ++i){
          printf("id: %d x: %d y: %d size x: %i size y: %i\n", object[i].id, object[i].position_on_image[0], 
          object[i].position_on_image[1], object[i].size_on_image[0],object[i].size_on_image[1]);
        }
        */
        
        
        for(int i = 0; i<num_objects; ++i)
        {
          if(object[i].id == obj.id)
          {
            state = 2;
          }
        }
        break;
        
        
      case 2:
        for(int i = 0; i<num_objects; ++i){
      
          //printf("id: %d x: %d y: %d size x: %i size y: %i\n", object[i].id, object[i].position_on_image[0], 
          //object[i].position_on_image[1], object[i].size_on_image[0],object[i].size_on_image[1]);
         
          
          if(object[i].id == obj.id){            
            
            // x focal length
            fx = 400/(2*atan(0.785/2)); //fov = 0.785, width = 400 pixels
            // y focal length
            fy = 240/(2*atan(0.785/2)); //fov = 0.785, height = 240 pixels    
            
            
            obj.theta_x = (object[i].position_on_image[0]-400.0/2.0)/(400.0)*0.785;
            obj.theta_y = (object[i].position_on_image[1]-240.0/2.0)/(240.0)*0.785;
            
            obj.x = 1*fx/object[i].position_on_image[0]; //maybe not gonna use it
            obj.y = 1.2*fy/object[i].position_on_image[1];
            
            // Exponential controller with PID
            double e[2] = {0, obj.y/tan(obj.theta_y+camera_yaw_disturbance)};
            
            // Position error
            double eP = sqrt(pow(e[0],2)+pow(e[1],2));
            
            // Orientation error
            double eOx = obj.theta_x;
            double eOy = obj.theta_y;
            
            // Linear speed controller
            double kP = v0*(1-exp(-alfa*pow(eP,2)))/eP;
            pitch_disturbance = -1.0*kP*eP;
            
            // Angular speed controller
            double eO_Dx = eOx - eO_1x;
            double eO_Dy = eOy - eO_1y;
            
            EOx = EOx + eOx;
            EOy = EOy + eOy;
            
            yaw_disturbance = -1.0*(kpOx*eOx + kiOx*EOx*timestep/1000.0 + kdOx*eO_Dx*1000.0/timestep);
            camera_yaw_disturbance = 1.0*(kpOy*eOy + kiOy*EOy*timestep/1000.0 + kdOy*eO_Dy*1000.0/timestep);
            
            eO_1x = eOx;
            eO_1y = eOy;
            
            if(yaw_disturbance > 1.3){yaw_disturbance = 2;}
            else if (yaw_disturbance < -1.3){yaw_disturbance = -2;}
            
            if(camera_yaw_disturbance > 1.5){camera_yaw_disturbance = 1.5;}
            else if (camera_yaw_disturbance < -1.5){camera_yaw_disturbance = -1.5;}
            
            
            
            printf("object detected. pitch: %f eP: %f\n",pitch_disturbance, eP);
            //printf("object detected. pitch: %f yaw: %f roll: %f\n",pitch_disturbance,yaw_disturbance, camera_yaw_disturbance);
            
            
            if(eP<EPSILON_DROP){state=3;}
    
          }
        }
        
        break;
        
      case 3:
        
        latch_position = 0.5;
        printf("PACKAGE DROP!");
        state = 1;
        break;
      
    }
    
    
    
    
    
    // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_velocity);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_velocity + camera_yaw_disturbance);
    
    // Latch
    wb_motor_set_position(latch_motor, latch_position);

    // Compute the roll, pitch, yaw and vertical inputs.
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    const double yaw_input = yaw_disturbance;
    const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    // Actuate the motors taking into consideration all the computed inputs.
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
    printf("v1: %.1f v2: %.1f v3: %.1f v4: %.1f",front_left_motor_input,front_right_motor_input,rear_left_motor_input,rear_right_motor_input);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}

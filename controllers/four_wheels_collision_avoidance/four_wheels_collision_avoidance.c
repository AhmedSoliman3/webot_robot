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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <webots/compass.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/receiver.h>
#include <webots/utils/system.h>

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"


enum BLOB_TYPE { RED, GREEN, BLUE, NONE };
// time in [ms] of a simulation step
#define SPEED 6
#define TIME_STEP 64
typedef enum { GPS, SUPERVISED } gps_mode_types;

#define RAD_TO_DEG 57.2958
#define NB_STEPS 100

  int width, height;
  int pause_counter = 0;
  int left_speed, right_speed;
  int ii, jj;
  int red, blue, green;
  const char *color_names[3] = {"red", "green", "blue"};
  const char *ansi_colors[3] = {ANSI_COLOR_RED, ANSI_COLOR_GREEN, ANSI_COLOR_BLUE};
  const char *filenames[3] = {"red_blob.png", "green_blob.png", "blue_blob.png"};
  enum BLOB_TYPE current_blob;
  
  
// source: https://ahrs.readthedocs.io/en/latest/filters/angular.html
double *integrate_gyro(const double *current_value, const double *gyro_values, double sample_time) {
  // compute rotation matrix from sample time and gyro rates
  const double identity_matrix[4][4] = {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
  const double gyro_matrix[4][4] = {{0.0, -gyro_values[0], -gyro_values[1], -gyro_values[2]},
                                    {gyro_values[0], 0.0, gyro_values[2], -gyro_values[1]},
                                    {gyro_values[1], -gyro_values[2], 0.0, gyro_values[0]},
                                    {gyro_values[2], gyro_values[1], -gyro_values[0], 0.0}};

  double rotation_matrix[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      rotation_matrix[i][j] = sample_time * gyro_matrix[i][j];
      rotation_matrix[i][j] += identity_matrix[i][j];
    }
  }

  // apply gyro rates to the current quaternion
  static double new_quaternion[4] = {0.0, 0.0, 0.0, 0.0};
  double q_norm = 0.0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++)
      new_quaternion[i] += rotation_matrix[i][j] * current_value[j];
    q_norm += pow(new_quaternion[i], 2.0);
  }

  // normalize new quaternion
  q_norm = sqrt(q_norm);
  for (int i = 0; i < 4; i++)
    new_quaternion[i] = new_quaternion[i] / q_norm;

  return new_quaternion;
}

// source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
double quaternion_to_roll(const double *quaternion) {
  const double sinr_cosp = 2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]);
  const double cosr_cosp = 1 - 2 * (pow(quaternion[1], 2.0) + pow(quaternion[2], 2.0));
  const double roll = atan2(sinr_cosp, cosr_cosp);
  return roll;
}

// source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
double quaternion_to_pitch(const double *quaternion) {
  const double sinp = 2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]);
  double pitch = 0.0;
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2.0, sinp);  // use pi/2 radians if out of range
  else
    pitch = asin(sinp);
  return pitch;
}

// source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
double quaternion_to_yaw(const double *quaternion) {
  const double siny_cosp = 2.0 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]);
  const double cosy_cosp = 1.0 - 2.0 * (pow(quaternion[2], 2.0) + pow(quaternion[3], 2.0));
  const double yaw = atan2(siny_cosp, cosy_cosp);
  return yaw;
}

// source: https://forum.arduino.cc/t/getting-pitch-and-roll-from-acceleromter-data/694148
double attitude_from_accelerometer(int axis, const double *accelerometer_values) {
  double output = 0.0;
  if (axis == 0)  // roll axis
    output = atan2(accelerometer_values[1], accelerometer_values[2]);
  else  // pitch axis
    output = atan2(-accelerometer_values[0], sqrt(pow(accelerometer_values[1], 2.0) + pow(accelerometer_values[2], 2.0)));
  return output;
}

// source: http://robo.sntiitk.in/2017/12/21/Beginners-Guide-to-IMU.html
double yaw_from_compass(double roll, double pitch, const double *compass_values) {
  const double mag_x =
    compass_values[0] * cos(pitch) + compass_values[1] * sin(roll) * sin(pitch) + compass_values[2] * cos(roll) * sin(pitch);
  const double mag_y = compass_values[1] * cos(roll) - compass_values[2] * sin(roll);
  const double yaw = atan2(mag_x, mag_y);

  return yaw;
}

void readCamera(WbDeviceTag camera,const int time_step)
{
  const unsigned char *image = wb_camera_get_image(camera);
    /* Decrement the pause_counter */
    if (pause_counter > 0)
      pause_counter--;

    /*
     * Case 1
     * A blob was found recently
     * The robot waits in front of it until pause_counter
     * is decremented enough
     */
    if (pause_counter > 640 / time_step) {
      left_speed = 0;
      right_speed = 0;
    }
    /*
     * Case 2
     * A blob was found quite recently
     * The robot begins to turn but don't analyse the image for a while,
     * otherwise the same blob would be found again
     */
    else if (pause_counter > 0) {
      left_speed = -SPEED;
      right_speed = SPEED;
    }
    /*
     * Case 3
     * The robot turns and analyse the camera image in order
     * to find a new blob
     */
    else if (!image) {  // image may be NULL if Robot.synchronization is FALSE
      left_speed = 0;
      right_speed = 0;
    } else {  // pause_counter == 0
      /* Reset the sums */
      red = 0;
      green = 0;
      blue = 0;

      /*
       * Here we analyse the image from the camera. The goal is to detect a
       * blob (a spot of color) of a defined color in the middle of our
       * screen.
       * In order to achieve that we simply parse the image pixels of the
       * center of the image, and sum the color components individually
       */
      for (ii = width / 3; ii < 2 * width / 3; ii++) {
        for (jj = height / 2; jj < 3 * height / 4; jj++) {
          red += wb_camera_image_get_red(image, width, ii, jj);
          blue += wb_camera_image_get_blue(image, width, ii, jj);
          green += wb_camera_image_get_green(image, width, ii, jj);
        }
      }

      /*
       * If a component is much more represented than the other ones,
       * a blob is detected
       */
      if ((red > 3 * green) && (red > 3 * blue))
        current_blob = RED;
      else if ((green > 3 * red) && (green > 3 * blue))
        current_blob = GREEN;
      else if ((blue > 3 * red) && (blue > 3 * green))
        current_blob = BLUE;
      else
        current_blob = NONE;

      /*
       * Case 3a
       * No blob is detected
       * the robot continues to turn
       */
      if (current_blob == NONE) {
        left_speed = -SPEED;
        right_speed = SPEED;
      }
      /*
       * Case 3b
       * A blob is detected
       * the robot stops, stores the image, and changes its state
       */
      else {
        left_speed = 0;
        right_speed = 0;
        printf("Looks like I found a %s%s%s blob.\n", ansi_colors[current_blob], color_names[current_blob], ANSI_COLOR_RESET);
        // compute the file path in the user directory
        char *filepath;
#ifdef _WIN32
        const char *user_directory = wbu_system_short_path(wbu_system_getenv("USERPROFILE"));
        filepath = (char *)malloc(strlen(user_directory) + 16);
        strcpy(filepath, user_directory);
        strcat(filepath, "\\");
#else
        const char *user_directory = wbu_system_getenv("HOME");
        filepath = (char *)malloc(strlen(user_directory) + 16);
        strcpy(filepath, user_directory);
        strcat(filepath, "/");
#endif
        strcat(filepath, filenames[current_blob]);
        wb_camera_save_image(camera, filepath, 100);
        free(filepath);
        pause_counter = 1280 / time_step;
      }
    }
}

double mean_error(const double *ground_truth, const double *estimation) {
  const double mean_error =
    (fabs(ground_truth[0] - estimation[0]) + fabs(ground_truth[1] - estimation[1]) + fabs(ground_truth[2] - estimation[2])) / 3;
  return mean_error;
}

// entree point of the controller
int main(int argc, char **argv) {

  // initialise the Webots API
  wb_robot_init();

  // internal variables
  int i;
  int avoid_obstacle_counter = 0;

  // initialise distance sensors
  WbDeviceTag ds[2];
  WbDeviceTag camera;
  
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }

  // initialise motors
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }

  /* get and enable GPS device */
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
    /* get and enable receiver */
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  // /* get a handler to the motors and set target position to infinity (speed control). */
  // WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  // WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  // wb_motor_set_position(left_motor, INFINITY);
  // wb_motor_set_position(right_motor, INFINITY);
  // wb_motor_set_velocity(left_motor, 0.0);
  // wb_motor_set_velocity(right_motor, 0.0);
  
    /* get key presses from keyboard */
  wb_keyboard_enable(TIME_STEP);
  
  const int time_step = wb_robot_get_basic_time_step();

  WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, time_step);

  WbDeviceTag imu_accelerometer = wb_robot_get_device("accelerometer");
  WbDeviceTag imu_gyro = wb_robot_get_device("gyro");
  WbDeviceTag imu_compass = wb_robot_get_device("compass");
  wb_accelerometer_enable(imu_accelerometer, time_step);
  wb_gyro_enable(imu_gyro, time_step);
  wb_compass_enable(imu_compass, time_step);
  double gyro_quaternion[4] = {1.0, 0.0, 0.0, 0.0};
  double absolute_attitude[3] = {0.0, 0.0, 0.0};
  double relative_attitude[3] = {0.0, 0.0, 0.0};

  // WbDeviceTag roll_motor = wb_robot_get_device("roll motor");
  // WbDeviceTag pitch_motor = wb_robot_get_device("pitch motor");
  // WbDeviceTag yaw_motor = wb_robot_get_device("yaw motor");


  /* Get the camera device, enable it, and store its width and height */
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);

  
  
  printf("Press 'G' to read the GPS device's position\n");
  printf("Press 'S' to read the Supervisor's position\n");
  printf("Press 'V' to read the GPS device's speed vector\n");
  // feedback loop
  while (wb_robot_step(TIME_STEP) != -1) {
  
      switch (wb_keyboard_get_key()) {
      case 'G': {
        const double *gps_values = wb_gps_get_values(gps);
        printf("GPS position: %.3f %.3f %.3f\n", gps_values[0], gps_values[1], gps_values[2]);
        break;
      }
      case 'S': {
        /* received nothing so far */
        if (wb_receiver_get_queue_length(receiver) == 0)
          break;

        /* destroy all packets but last one */
        while (wb_receiver_get_queue_length(receiver) > 1)
          wb_receiver_next_packet(receiver);

        /* read the last packet */
        const double *buffer = wb_receiver_get_data(receiver);
        printf("Supervisor position: %.3f %.3f %.3f\n", buffer[0], buffer[1], buffer[2]);
        break;
      }
      case 'V': {
        const double *speed_vector_values = wb_gps_get_speed_vector(gps);
        printf("GPS speed vector: %.3f %.3f %.3f\n", speed_vector_values[0], speed_vector_values[1], speed_vector_values[2]);
        break;
      }
      default:
        break;
    }

  readCamera(camera,TIME_STEP);


    
        // init speeds
    double left_speed = 1.0;
    double right_speed = 1.0;

    if (avoid_obstacle_counter > 0) {
      avoid_obstacle_counter--;
      left_speed = 1.0;
      right_speed = -1.0;
    } else {
      // read sensors outputs
      double ds_values[2];
      for (i = 0; i < 2; i++)
        ds_values[i] = wb_distance_sensor_get_value(ds[i]);

      // increase counter in case of obstacle
      if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
        avoid_obstacle_counter = 100;
    }
    
    // write actuators inputs
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    wb_motor_set_velocity(wheels[2], left_speed);
    wb_motor_set_velocity(wheels[3], right_speed);
  }

  // cleanup the Webots API
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}

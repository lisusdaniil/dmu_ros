//
// Created by leutrim on 11/04/18.
//

#ifndef DMU_ROS_DMU11_H
#define DMU_ROS_DMU11_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <termios.h>
#include <string>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>


class DMU11
{
    ros::Publisher imu_publisher_;
    std::string device_;
    std::string frame_id_;
    double rate_;

    // DMU11 data package
    struct dmu_package
    {
        //                              // WORD
        int16_t msg_count;              //  1
        float axis_x_rate;              //  2-3
        float axis_x_acc;               //  4-5
        float axis_y_rate;              //  6-7
        float axis_y_acc;               //  8-9
        float axis_z_rate;              //  10-11
        float axis_z_acc;               //  12-13
        //--Reserved--//
        float average_imu_temp;         //  16-17
        //-----------------------------------------
        float axis_x_delta_theta;       //  18-19
        float axis_x_delta_vel;         //  20-21
        float axis_y_delta_theta;       //  22-23
        float axis_y_delta_vel;         //  24-25
        float axis_z_delta_theta;       //  26-27
        float axis_z_delta_vel;         //  28-29
        //-----------------------------------------
        int16_t system_startup_flags;   //  30
        int16_t system_operat_flags;    //  31

    } package_;


    //Constants
    const double g_ = 9.80665;
    int16_t header_ = 0x55aa;
    int file_descriptor_;
    struct termios defaults_;

public:
//IMU ROS format
    sensor_msgs::Imu raw_data_;

    double d_roll_ = 0;
    double d_pitch_ = 0;
    double d_yaw_ = 0;

/**
 * @brief Constructor
 * @param nh
 */
    DMU11(ros::NodeHandle &nh);

/**
 * @brief Open device
 * @retval 0 Success
 * @retval -1 Failure
 */
    int openPort();

/**
 * @brief Read the data received on serial port
 */
    void update();

/**
 * @brief Close the device
 */
    void closePort();

/**
 * @brief change big endian 2 byte into short
 * @param data Head pointer to the data
 * @retrun converted value
 */
    int16_t big_endian_to_short(unsigned char *data);

/**
 * @brief change big endian short into float
 * @param data Head pointer to the data
 * @retrun converted value
 */
    float short_to_float(int16_t *data);

/**
* @brief Gets the raw data and converts them to standard ROS IMU message
* @param int16buff
*/
    void doParsing(int16_t *int16buff);


    virtual ~DMU11();

};


#endif //DMU_ROS_DMU11_H

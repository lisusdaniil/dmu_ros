//
// Created by leutrim on 11/04/18.
//

#ifndef DMU_ROS_DMU11_H
#define DMU_ROS_DMU11_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <termios.h>
#include <string>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>


class DMU11
{
    ros::Publisher imuPub;
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
        //--Reserved--
        int16_t checksum;               //  33
    } package_;
    //IMU ROS format
    sensor_msgs::Imu raw_data_;

    //Constants
    const double g_ = 9.80665;
    int16_t header_ = 0x55aa;
    int file_descriptor_;
    struct termios defaults_;

public:
    volatile sig_atomic_t terminate_flag_;

    union bit16
    {
        unsigned char chbuff[2];
        int16_t int16;
    };

    union bit32
    {
        unsigned char chbuff[4];
        float fbuff;
    };

    DMU11(ros::NodeHandle &nh);

    int openPort(std::string device_path);

    int getProductID(int16_t &pid);

    void update();

    void doParsing(int16_t *int16buff);

    void closePort();

    int16_t big_endian_to_short(unsigned char *data);

    float short_to_float(int16_t *data);

    virtual ~DMU11();

};


#endif //DMU_ROS_DMU11_H

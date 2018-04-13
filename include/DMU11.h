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

public:

    int16_t header_;


    union bit16{
        unsigned char chbuff[2];
        int16_t int16;
    };

    union bit32{
        unsigned char chbuff[4];
        float fbuff;
    };

    struct dmu_package
    {
        //                              // WORD
        int16_t header;                 //  0
        int16_t msg_count;              //  1
        float axis_x_rate;              //  2-3
        float axis_x_acc;               //  4-5
        float axis_y_rate;              //  6-7
        float axis_y_acc;               //  8-9
        float axis_z_rate;              //  10-11
        float axis_z_acc;               //  12-13

        //
        float axis_x_delta_theta;       //  18-19
        float axis_x_delta_vel;         //  20-21
        float axis_y_delta_theta;       //  22-23
        float axis_y_delta_vel;         //  24-25
        float axis_z_delta_theta;       //  26-27
        float axis_z_delta_vel;         //  28-29

    };

    int file_descriptor_;
    struct termios defaults_;
    volatile sig_atomic_t terminate_flag_;

    DMU11(ros::NodeHandle &nh);

    int openPort(std::string device_path);

    int getProductID(int16_t &pid);

    void update();

    void closePort();

    int16_t big_endian_to_short( char *data);
    float big_endian_to_float(unsigned char *data);

    virtual ~DMU11();

};


#endif //DMU_ROS_DMU11_H

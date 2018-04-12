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
    int file_descriptor_;
    struct termios defaults_;
    volatile sig_atomic_t terminate_flag_;

    DMU11(ros::NodeHandle &nh);

    int openPort(std::string device_path);

    int getProductID(int16_t& pid);

    void update();

    void closePort();

    int16_t big_endian_to_short(unsigned char *data);


    virtual ~DMU11();

};


#endif //DMU_ROS_DMU11_H

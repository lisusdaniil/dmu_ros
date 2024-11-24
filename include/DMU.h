//
// Created by leutrim on 11/04/18.
//

#ifndef DMU_ROS_DMU_H
#define DMU_ROS_DMU_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <dmu_ros/DMURaw.h>
#include <dmu_ros/DMUCounter.h>
#include <tf/tf.h>
#include <termios.h>
#include <string>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>


class DMU
{
    ros::Publisher imu_publisher_;
    ros::Publisher dmu_raw_publisher_;
    ros::Publisher imu_counter_publisher_;
    std::string device_;
    std::string frame_id_;

    dmu_ros::DMURaw raw_package_; //Custom message containing all DMU raw data.
    dmu_ros::DMUCounter imu_counter_;

    //Constants
    const double g_ = 9.80665;
    int16_t header_ = 0x55aa;
    int file_descriptor_;
    struct termios defaults_;

public:
//IMU ROS format
    sensor_msgs::Imu imu_raw_;
    double rate_;

    int32_t msg_count_ = 0;
    int16_t prev_msg_count_ = 0;

    double roll_ = 0;
    double pitch_ = 0;
    double yaw_ = 0;

/**
 * @brief Constructor
 * @param nh
 */
    DMU(ros::NodeHandle &nh);

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


    virtual ~DMU();

};


#endif //DMU_ROS_DMU_H

/*
 *  Copyright (C) 2018 Leo Muhendislik, Leutrim Gruda
 *
 *  License: Modified BSD Software License Agreement
 *
 */


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>


void imuCallback(const sensor_msgs::ImuConstPtr &imu_in)
{
    double d_roll, d_pitch, d_yaw;
    static double roll, pitch, yaw;
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::Quaternion q;
    q.setX(imu_in->orientation.x);
    q.setY(imu_in->orientation.y);
    q.setZ(imu_in->orientation.z);
    q.setW(imu_in->orientation.w);

    tf::Matrix3x3 m(q);

    m.getRPY(d_roll, d_pitch, d_yaw);

    roll += d_roll;
    pitch += d_pitch;
    yaw += d_yaw;

    q.setRPY(roll, pitch, yaw);

    transform.setOrigin(tf::Vector3(0, 0, 0));
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu"));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simulation");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data_raw", 10, imuCallback);

    while (ros::ok())
        ros::spin();

    return 0;
}
//
// Created by leutrim on 16/05/18.
//

//
// Created by leutrim on 16/05/18.
//
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

static bool init_gyro = false;
static bool init_acc = false;

void gyro_cb(const sensor_msgs::ImuConstPtr &gyro_in)
{
    double dth_x, dth_y, dth_z;
    static ros::Time last_gyro;

    if (!init_gyro)
    {
        init_gyro = true;
        last_gyro = gyro_in->header.stamp;
        return;
    }


    double sampling_time = (gyro_in->header.stamp - last_gyro).toSec();

    dth_x = (gyro_in->linear_acceleration.x * 180 / M_PI) * sampling_time;
    dth_y = (gyro_in->linear_acceleration.y * 180 / M_PI) * sampling_time;
    dth_z = (gyro_in->linear_acceleration.z * 180 / M_PI) * sampling_time;

    std::cout << "Delta Theta X: " << gyro_in->angular_velocity.x * 180 / M_PI
              << " Delta Theta Y: " << gyro_in->angular_velocity.y * 180 / M_PI
              << " Delta Theta Z: " << gyro_in->angular_velocity.z * 180 / M_PI << std::endl;
    std::cout << "Delta Computed Theta X: " << dth_x
              << " Delta Computed Theta Y: " << dth_y
              << " Delta Computed Theta Z: " << dth_z << std::endl << std::endl;

    last_gyro = gyro_in->header.stamp;
}

void acc_cb(const sensor_msgs::ImuConstPtr &acc_in)
{
    double dv_x, dv_y, dv_z;
    static ros::Time last_acc;

    if (!init_acc)
    {
        init_acc = true;
        last_acc = acc_in->header.stamp;
        return;
    }


    double sampling_time = (acc_in->header.stamp - last_acc).toSec();

    dv_x = acc_in->linear_acceleration.x * sampling_time;
    dv_y = acc_in->linear_acceleration.y * sampling_time;
    dv_z = acc_in->linear_acceleration.z * sampling_time;

    std::cout << "Delta Velocity X: " << acc_in->angular_velocity.x
              << " Delta Velocity Y: " << acc_in->angular_velocity.y
              << " Delta Velocity Z: " << acc_in->angular_velocity.z << std::endl;
    std::cout << "Delta Computed Velocity X: " << dv_x
              << " Delta Computed Velocity Y: " << dv_y
              << " Delta Computed Velocity Z: " << dv_z << std::endl << std::endl;

    last_acc = acc_in->header.stamp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmu");
    ros::NodeHandle nh;

    ros::Subscriber gyro_sub = nh.subscribe("/imu/data_raw_gyro", 10, gyro_cb);
    ros::Subscriber acc_sub = nh.subscribe("/imu/data_raw_acc", 10, acc_cb);

    while (ros::ok())
        ros::spin();

    return 0;
}
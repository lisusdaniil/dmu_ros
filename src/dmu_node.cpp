#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <DMU.h>

std::shared_ptr<DMU> imu;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dmu_node");
    ros::NodeHandle nh;

    imu = std::shared_ptr<DMU>(new DMU(nh));

    imu->openPort();

    usleep(100000);

    ros::Rate rate(imu->rate_);
    while (ros::ok()) {
        imu->update();
        ros::spinOnce();
        rate.sleep();
    }

    imu->closePort();


    return 0;
}
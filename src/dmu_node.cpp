#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <DMU11.h>

std::shared_ptr<DMU11> imu;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dmu_node");
    ros::NodeHandle nh;

    imu = std::shared_ptr<DMU11>(new DMU11(nh));

    imu->openPort();

    usleep(100000);


    ros::Rate rate(200);
    while (ros::ok())
    {
        imu->update();
        ros::spinOnce();
        rate.sleep();
    }

    imu->closePort();


    return 0;
}
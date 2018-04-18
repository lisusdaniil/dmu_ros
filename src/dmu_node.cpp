#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <DMU11.h>
#include <memory>


std::shared_ptr<DMU11> imu;

void signalHandler(int sig)
{
    imu->terminate_flag_ = 1;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dmu_ros");
    ros::NodeHandle nh;

    imu = std::shared_ptr<DMU11>(new DMU11(nh));

    signal(SIGINT, signalHandler);

    std::string device_path;

    nh.param("device", device_path, std::string("/dev/ttyUSB0"));

    imu->openPort(device_path);

    usleep(100000);

    ros::Rate rate(200);

    while (ros::ok() && !imu->terminate_flag_)
    {
        imu->update();
        ros::spinOnce();
        rate.sleep();
    }

    int16_t pid = 0;
//    imu.getProductID(pid);
    ROS_INFO("Product ID: %x\n", pid);

    imu->closePort();


    return 0;
}
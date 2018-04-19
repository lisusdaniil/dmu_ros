#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <DMU11.h>


std::shared_ptr<DMU11> imu;

void signalHandler(int sig)
{
    imu->terminate_flag_ = 1;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dmu_ros");
    ros::NodeHandle nh;

    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform;

    imu = std::shared_ptr<DMU11>(new DMU11(nh));

    signal(SIGINT, signalHandler);

    std::string device_path("/dev/ttyUSB0");

//    nh.getParam("/dmu_ros_node/device", device_path);

    imu->openPort(device_path);

    usleep(100000);

    ros::Rate rate(200);

    while (ros::ok() && !imu->terminate_flag_)
    {
        static double roll, pitch, yaw;

        imu->update();

        tf::Quaternion q;

        roll += imu->d_roll_;
        pitch += imu->d_pitch_;
        yaw += imu->d_yaw_;

        q.setRPY(roll, pitch, yaw);

        transform.setOrigin(tf::Vector3(0, 0, 0));
        transform.setRotation(q);
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu"));

        ros::spinOnce();
        rate.sleep();
    }

    imu->closePort();


    return 0;
}
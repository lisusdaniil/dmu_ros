//
// Created by leutrim on 11/04/18.
//

#include <DMU11.h>
#include <bitset>


DMU11::DMU11(ros::NodeHandle &nh)
{
    terminate_flag_ = 0;

    // Read parameters
    nh.param("device", device_, std::string("/dev/ttyUSB0"));
    nh.param("frame_id", frame_id_, std::string("imu"));
    nh.param("rate", rate_, 100.0);

    imuPub = nh.advertise<sensor_msgs::Imu>("data/raw", 10);
}



int DMU11::openPort(std::string device_path)
{
    int trials = 0;
    do
    {
        if (trials++ == 0)
            std::cout << "Opening serial port: " << device_path.c_str() << " \n";
        else
            std::cout << "Couldn't open serial port... \n";
        file_descriptor_ = open(device_path.c_str(), O_RDWR | O_NOCTTY);
        int opts = fcntl(file_descriptor_, F_GETFL);
        opts = opts & (O_NONBLOCK);
        fcntl(file_descriptor_, F_SETFL, opts);    /*configuration the port*/
    } while (file_descriptor_ == -1);


    int diag;
    trials = 0;
    do
    {
        if (trials++ != 0)
            std::cout << "Couldn't get the current state.\n";
        diag = tcgetattr(file_descriptor_, &defaults_);
    } while (diag == -1 && errno == EINTR);
    std::cout << "Current state is read.\n";


    if (cfsetispeed(&defaults_, B460800) < 0 || cfsetospeed(&defaults_, B460800) < 0)
    {
        std::cout << "Couldnt set the desired baud rate \n";
        return -1;
    }


    // Define com port options
    //
    defaults_.c_cflag |= (CLOCAL | CREAD);    // Enable the receiver and set local mode...
    defaults_.c_cflag &= ~(PARENB | CSIZE);  // No parity, mask character size bits
    defaults_.c_cflag |= CSTOPB;        //2 stop bits
    defaults_.c_cflag |= CS8;            // Select 8 data bits
    defaults_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    defaults_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG /*| IEXTEN | ECHONL*/);
    defaults_.c_oflag &= ~OPOST;

    defaults_.c_cc[VMIN] = 68;       //Minimum - two bytes
    defaults_.c_cc[VTIME] = 0;

    if (tcsetattr(file_descriptor_, TCSANOW, &defaults_) != 0)
    {
        std::cout << "Failed to set attributes to the serial port.\n";
        return -1;
    }

    usleep(10000);

    unsigned char buff[3] = {0};

    tcflush(file_descriptor_,TCIFLUSH);
    if (tcdrain(file_descriptor_) < 0)
    {
        perror("flush");
        return -1;
    }
/*    // Turn message stream off sequence
    buff[0] = 0x04;
    buff[1] = 0x01;
    buff[2] = 0x00;

    int size = write(file_descriptor_, buff, 3);
    if (size != 3)
    {
        perror("Stop stream");
        return -1;
    }

    if (tcdrain(file_descriptor_) < 0)
    {
        perror("Stop stream");
        return -1;
    }*/

    // Turn message stream on sequence
    buff[0] = 0x04;
    buff[1] = 0x01;
    buff[2] = 0x01;

    int size = write(file_descriptor_, buff, 3);
    if (size != 3)
    {
        perror("Start stream");
        return -1;
    }

    if (tcdrain(file_descriptor_) < 0)
    {
        perror("Start stream");
        return -1;
    }

    return 0;
}

void DMU11::update()
{

    unsigned char buff[68] = {0};

    int size = read(file_descriptor_, buff, 68);
    if (size != 68)
    {
        perror("Partial Package Received.");
        return;
    }
    else
    {

        int16_t computed_checksum = 0;
        int16_t input16 = big_endian_to_short(&buff[0]);
        int16_t int16buff[34] = {0};

        if (input16 == header_)
        {
            for (int i = 0; i < 66; i += 2)
            {
                int16buff[i / 2] = big_endian_to_short(&buff[i]);
                computed_checksum += int16buff[i / 2];
            }

            computed_checksum = ~computed_checksum + 0x0001;

            int16_t checksum = big_endian_to_short(&buff[66]);

            if (checksum == computed_checksum)
                doParsing(&int16buff[0]);
            else
            {
                std::cout << "Package is corrupt...\n";
                std::cout << "Checksum: " << checksum << "\n";
                std::cout << "Computed Checksum: " << computed_checksum << "\n";
                return;
            }
        }
        else
        {

            std::cout << "********************\n";
            std::cout << "******Restart*******\n";
            std::cout << "********************\n";
            unsigned char buff1[3] = {0};
            buff1[0] = 0x04;
            buff1[1] = 0x01;
            buff1[2] = 0x00;  // Turn message stream off
            size = write(file_descriptor_, buff1, 3);
            if (size != 3)
            {
                perror("Start stream");
            }

            if (tcdrain(file_descriptor_) < 0)
            {
                perror("Start stream");
            }

            usleep(100000);
            tcflush(file_descriptor_, TCIFLUSH);

            buff1[0] = 0x04;
            buff1[1] = 0x01;
            buff1[2] = 0x01;  // Turn message stream on
            size = write(file_descriptor_, buff1, 3);
            if (size != 3)
            {
                perror("Start stream");
            }

            if (tcdrain(file_descriptor_) < 0)
            {
                perror("Start stream");
            }
            usleep(100000);

        }
    }

}

void DMU11::closePort()
{
    if (tcsetattr(file_descriptor_, TCSANOW, &defaults_) < 0)
    {
        perror("closePort");
    }
    int diag;
    do
    {
        diag = close(file_descriptor_);
    } while (diag == -1);

    std::cout << "Serial Port got closed.";

}



int16_t DMU11::big_endian_to_short(unsigned char *data)
{
    unsigned char buff[2] = {data[1], data[0]};
    return *reinterpret_cast<int16_t *>(buff);
}

float DMU11::short_to_float(int16_t *data)
{
    int16_t buff[2] = {data[1], data[0]};
    return *reinterpret_cast<float *>(buff);
}

void DMU11::doParsing(int16_t *int16buff)
{
    package_.axis_x_acc = short_to_float(&int16buff[4]) * g_; // We divide by g_ to convert from g's into m/s^2
    package_.axis_y_acc = short_to_float(&int16buff[8]) * g_;
    package_.axis_z_acc = short_to_float(&int16buff[12]) * g_;

    package_.axis_x_rate = short_to_float(&int16buff[2]) * M_PI / 180; //Conver to rad/s
    package_.axis_y_rate = short_to_float(&int16buff[6]) * M_PI / 180;
    package_.axis_z_rate = short_to_float(&int16buff[10]) * M_PI / 180;

    package_.axis_x_delta_vel = short_to_float(&int16buff[20]);
    package_.axis_y_delta_vel = short_to_float(&int16buff[24]);
    package_.axis_z_delta_vel = short_to_float(&int16buff[28]);

    package_.axis_x_delta_theta = short_to_float(&int16buff[18]) * M_PI / 180;
    package_.axis_y_delta_theta = short_to_float(&int16buff[22]) * M_PI / 180;
    package_.axis_z_delta_theta = short_to_float(&int16buff[26]) * M_PI / 180;

    package_.system_startup_flags = int16buff[30];
    package_.system_operat_flags = int16buff[31];

    raw_data_.header.frame_id = frame_id_;
    raw_data_.header.stamp = ros::Time::now();

    raw_data_.linear_acceleration.x = package_.axis_x_acc;
    raw_data_.linear_acceleration.y = package_.axis_y_acc;
    raw_data_.linear_acceleration.z = package_.axis_z_acc;

    raw_data_.angular_velocity.x = package_.axis_x_rate;
    raw_data_.angular_velocity.y = package_.axis_y_rate;
    raw_data_.angular_velocity.z = package_.axis_z_rate;

    d_roll_ = package_.axis_x_delta_theta;
    d_pitch_ = package_.axis_y_delta_theta;
    d_yaw_ = package_.axis_z_delta_theta;

    tf::Quaternion q;
    q.setRPY(d_roll_, d_pitch_, d_yaw_);

    raw_data_.orientation.x = q.x();
    raw_data_.orientation.y = q.y();
    raw_data_.orientation.z = q.z();
    raw_data_.orientation.w = q.w();

    imuPub.publish(raw_data_);

}


DMU11::~DMU11()
{

}





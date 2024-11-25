//
// Created by leutrim on 11/04/18. Edited by lisusdaniil on 05/25/24
//

#include <DMU.h>
#include <ros/console.h>

DMU::DMU(ros::NodeHandle &nh) {
    bool params = false;
    // Read parameters
    while (!params)
        try
        {
            params = nh.getParam("dmu_node/device", device_);
        }
        catch (ros::Exception &exception)
        {
            ROS_ERROR("Error %s", exception.what());
        }

    nh.param("dmu_node/frame_id", frame_id_, std::string("imu"));
    nh.param("dmu_node/rate", rate_, 200.0);

    imu_publisher_ = nh.advertise<sensor_msgs::Imu>("dmu/imu", 10);
    dmu_raw_publisher_ = nh.advertise<dmu_ros::DMURaw>("dmu/data_raw", 10);
    imu_counter_publisher_ = nh.advertise<dmu_ros::DMUCounter>("dmu/imu_counter", 10);
}


int DMU::openPort() {
    int trials = 0;
    do
    {
        if (trials++ == 0)
            std::cout << "Opening serial port: " << device_.c_str() << " \n";
        else
            std::cout << "Couldn't open serial port... \n";
        file_descriptor_ = open(device_.c_str(), O_RDWR | O_NOCTTY);
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

    if (cfsetispeed(&defaults_, B460800) < 0 || cfsetospeed(&defaults_, B460800) < 0)
    {
        std::cout << "Couldnt set the desired baud rate \n";
        return -1;
    }


    // Define com port options
    defaults_.c_cflag |= (CLOCAL | CREAD);    // Enable the receiver and set local mode...
    defaults_.c_cflag &= ~(PARENB | CSIZE);  // No parity, mask character size bits
    defaults_.c_cflag |= CSTOPB;        //2 stop bits
    defaults_.c_cflag |= CS8;            // Select 8 data bits
    defaults_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    defaults_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG /*| IEXTEN | ECHONL*/);
    defaults_.c_oflag &= ~OPOST;

    defaults_.c_cc[VMIN] = 2;       //Minimum - two bytes
    defaults_.c_cc[VTIME] = 0;

    if (tcsetattr(file_descriptor_, TCSANOW, &defaults_) != 0)
    {
        std::cout << "Failed to set attributes to the serial port.\n";
        return -1;
    }

    usleep(10000);

    // Restart the stream
    unsigned char buff[3] = {0};
    buff[0] = 0x04;
    buff[1] = 0x01;
    buff[2] = 0x00;  // Turn message stream off
    int size = write(file_descriptor_, buff, 3);
    if (size != 3)
    {
        perror("DMU: Stop stream");
    }

    if (tcdrain(file_descriptor_) < 0)
    {
        perror("DMU: Stop stream");
    }

    tcflush(file_descriptor_, TCIFLUSH);
    if (tcdrain(file_descriptor_) < 0)
    {
        perror("DMU: flush");
        return -1;
    }

    usleep(100000);
    tcflush(file_descriptor_, TCIFLUSH);

    buff[0] = 0x04;
    buff[1] = 0x01;
    buff[2] = 0x01;  // Turn message stream on
    size = write(file_descriptor_, buff, 3);
    if (size != 3)
    {
        perror("DMU: Start stream");
    }

    if (tcdrain(file_descriptor_) < 0)
    {
        perror("DMU: Start stream");
    }

    msg_count_ = 0;

    std::cout << "Started reading data from sensor...\n";

    return 0;
}


void DMU::update() {
    unsigned char buff[68] = {0};
    int size = 0;

    // Loop to find the correct header
    int num_loops = 0;
    while (true) {
        // Prevent inifnite loop
        if (num_loops > 68) {
            perror("DMU: Could not find header.");
            break;
        }

        // Read one byte at a time to search for the header
        unsigned char byte;
        int bytesRead = read(file_descriptor_, &byte, 1);

        if (bytesRead <= 0) {
            perror("DMU: Error reading byte or stream ended.");
            return;
        }

        // Check if this byte matches the first byte of the header
        if (byte == (header_ >> 8)) {  // Compare MSB of header
            unsigned char nextByte;
            bytesRead = read(file_descriptor_, &nextByte, 1);

            if (bytesRead <= 0) {
                perror("DMU: Error reading second byte of header.");
                return;
            }

            // Check if the second byte matches the LSB of the header
            if (nextByte == (header_ & 0xFF)) {  // Compare LSB of header
                // Store the header bytes
                buff[0] = byte;
                buff[1] = nextByte;
                size = 2;

                // Read the remaining 66 bytes
                while (size < 68) {
                    int bytesRemaining = 68 - size;
                    bytesRead = read(file_descriptor_, buff + size, bytesRemaining);

                    if (bytesRead <= 0) {
                        perror("DMU: Error reading remaining bytes.");
                        return;
                    }

                    size += bytesRead;
                }

                break;  // Exit the header search loop
            }
        }

        num_loops++;
    }

    // Validate and parse the package
    int16_t computed_checksum = 0;
    int16_t int16buff[34] = {0};

    for (int i = 0; i < 66; i += 2) {
        int16buff[i / 2] = big_endian_to_short(&buff[i]);
        computed_checksum += int16buff[i / 2];
    }

    computed_checksum = ~computed_checksum + 0x0001;
    int16_t checksum = big_endian_to_short(&buff[66]);

    if (checksum == computed_checksum) {
        doParsing(&int16buff[0]);
    } else {
        perror("DMU: Corrupted package.");
    }
}


void DMU::closePort() {
    if (tcsetattr(file_descriptor_, TCSANOW, &defaults_) < 0)
    {
        perror("DMU: closePort");
    }
    int diag;
    do
    {
        diag = close(file_descriptor_);
    } while (diag == -1);

    std::cout << "Serial Port got closed.";
}


int16_t DMU::big_endian_to_short(unsigned char *data) {
    unsigned char buff[2] = {data[1], data[0]};
    return *reinterpret_cast<int16_t *>(buff);
}


float DMU::short_to_float(int16_t *data) {
    int16_t buff[2] = {data[1], data[0]};
    return *reinterpret_cast<float *>(buff);
}


void DMU::doParsing(int16_t *int16buff) {
    raw_package_.header.stamp = ros::Time::now();
    raw_package_.header.frame_id = frame_id_;
    raw_package_.msg_count = int16buff[1];
    // std::cerr << int16buff[1] << std::endl;
    // perror("test");

    imu_raw_.header.stamp = raw_package_.header.stamp;
    imu_raw_.header.frame_id = frame_id_;

    // Accelerometer
    raw_package_.linear_acceleration.x = short_to_float(&int16buff[4]);
    raw_package_.linear_acceleration.y = short_to_float(&int16buff[8]);
    raw_package_.linear_acceleration.z = short_to_float(&int16buff[12]);

    imu_raw_.linear_acceleration.x =
            raw_package_.linear_acceleration.x * g_; // We multiply by g_ to convert from g's into m/s^2
    imu_raw_.linear_acceleration.y = raw_package_.linear_acceleration.y * g_;
    imu_raw_.linear_acceleration.z = raw_package_.linear_acceleration.z * g_;

    // Gyro Rates
    raw_package_.angular_rate.x = short_to_float(&int16buff[2]);
    raw_package_.angular_rate.y = short_to_float(&int16buff[6]);
    raw_package_.angular_rate.z = short_to_float(&int16buff[10]);

    imu_raw_.angular_velocity.x = raw_package_.angular_rate.x * M_PI / 180; //Convert to rad/s
    imu_raw_.angular_velocity.y = raw_package_.angular_rate.y * M_PI / 180;
    imu_raw_.angular_velocity.z = raw_package_.angular_rate.z * M_PI / 180;

    // Delta thetas
    raw_package_.delta_theta.x = short_to_float(&int16buff[18]);
    raw_package_.delta_theta.y = short_to_float(&int16buff[22]);
    raw_package_.delta_theta.z = short_to_float(&int16buff[26]);

    // Delta velocities
    raw_package_.delta_velocity.x = short_to_float(&int16buff[20]);
    raw_package_.delta_velocity.y = short_to_float(&int16buff[24]);
    raw_package_.delta_velocity.z = short_to_float(&int16buff[28]);


    raw_package_.system_startup_flags = int16buff[30];
    raw_package_.system_operat_flags = int16buff[31];


    roll_ += raw_package_.delta_theta.x * M_PI / 180;
    pitch_ += raw_package_.delta_theta.y * M_PI / 180;
    yaw_ += raw_package_.delta_theta.z * M_PI / 180;

    tf::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);

    imu_raw_.orientation.x = q.x();
    imu_raw_.orientation.y = q.y();
    imu_raw_.orientation.z = q.z();
    imu_raw_.orientation.w = q.w();

    // Process msg_counter
    msg_count_ += abs(abs(raw_package_.msg_count) - abs(prev_msg_count_));
    prev_msg_count_ = raw_package_.msg_count;

    // Fill out IMU Counter topic
    imu_counter_.header.stamp = raw_package_.header.stamp;
    imu_counter_.header.frame_id = frame_id_;
    imu_counter_.msg_count = msg_count_;
    imu_counter_.system_startup_flags = raw_package_.system_startup_flags;
    imu_counter_.system_operat_flags = raw_package_.system_operat_flags;
    imu_counter_.imu = imu_raw_;

    imu_publisher_.publish(imu_raw_);
    dmu_raw_publisher_.publish(raw_package_);
    imu_counter_publisher_.publish(imu_counter_);
}


DMU::~DMU() {}
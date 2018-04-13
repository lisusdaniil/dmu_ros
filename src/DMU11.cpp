//
// Created by leutrim on 11/04/18.
//

#include <DMU11.h>


DMU11::DMU11(ros::NodeHandle &nh)
{
    header_ = 0x55aa;
    terminate_flag_ = 0;
    imuPub = nh.advertise<sensor_msgs::Imu>("data/raw", 10);
}

DMU11::~DMU11()
{

}


/**
 * @brief Open device
 * @param device Device file name (/dev/ttyUSB*)
 * @retval 0 Success
 * @retval -1 Failure
 */
int DMU11::openPort(std::string device_path)
{
    int trials = 0;
    do
    {
        if (trials++ == 0)
            std::cout << "Opening serial port: " << device_path.c_str() << " \n";
        else
            std::cout << "Still couldn't open \n";
        file_descriptor_ = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        fcntl(file_descriptor_, F_SETFL, FNDELAY);    /*configuration the port*/
//        file_descriptor_ = open(device_path.c_str(), O_RDWR | O_NOCTTY);

    } while (file_descriptor_ == -1);

    if (isatty(file_descriptor_))
        std::cout << "Yes, it is a tty.\n\n";

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
        std::cout << "Couldnt set the desired baud rate \n";
    else
        std::cout << "Baud rate is set accordingly \n";


    /**********************************
    *
    *     no parity example
    *
    **********************************/
/*    // PARENB is enable parity bit
    // so this disables the parity bit
    defaults_.c_cflag &= ~PARENB;

    // CSTOPB means 2 stop bits
    defaults_.c_cflag &= CSTOPB;


    // CS8 means 8-bits per work
    defaults_.c_cflag |= CS8;

    defaults_.c_cflag |= (CLOCAL | CREAD);*/

/*    defaults_.c_iflag = 0;
    defaults_.c_oflag = 0;
    defaults_.c_cflag = CS8 | CREAD | CLOCAL;           // 8n1, see termios.h for more information
    defaults_.c_lflag = 0;*/


    //SBG configs

    //
    // Define com port options
    //
    defaults_.c_cflag |= (CLOCAL | CREAD);    // Enable the receiver and set local mode...
    defaults_.c_cflag &= ~(PARENB | CSTOPB | CSIZE);  // No parity, 1 stop bit, mask character size bits
    defaults_.c_cflag |= CS8;            // Select 8 data bits
    defaults_.c_cflag &= ~CRTSCTS;        // Disable Hardware flow control

    //
    // Disable software flow control
    //
    defaults_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

    //
    // We would like raw input
    //
    defaults_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG /*| IEXTEN | ECHONL*/);
    defaults_.c_oflag &= ~OPOST;

    //
    // Set our timeout to 0
    //
    defaults_.c_cc[VMIN] = 0;
    defaults_.c_cc[VTIME] = 1;

    if (tcsetattr(file_descriptor_, TCSAFLUSH, &defaults_) != 0)
        std::cout << "Failed to set attributes to the serial port.\n";
    else
        std::cout << "Desired configurations are set to serial port. \n";


    unsigned char buff0[3] = {0};
    buff0[0] = 0x34;
    buff0[1] = 0x31;
    buff0[2] = 0x30;  // Turn message stream off

    int size = write(file_descriptor_, buff0, 3);
    if (size != 3)
    {
        perror("Stop stream");
    }

    if (tcdrain(file_descriptor_) < 0)
    {
        perror("Stop stream");
    }


    usleep(100000);

    unsigned char buff1[3] = {0};
    buff1[0] = 0x34;
    buff1[1] = 0x31;
    buff1[2] = 0x31;  // Turn message stream on

    size = write(file_descriptor_, buff1, 3);
    if (size != 3)
    {
        perror("Start stream");
    }

    if (tcdrain(file_descriptor_) < 0)
    {
        perror("Start stream");
    }

    return 0;
}

/**
 * @brief Close the device
 */
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


/**
* @param data Product ID (0x4056)
* @retval 0 Success
* @retval -1 Failed
*/
int DMU11::getProductID(int16_t &pid)
{
    // get product ID
    int r;
     char buff[20];

    // Sending data
    buff[0] = 0x04;
    buff[1] = 0x01;
    buff[2] = 0x01;
    int size = write(file_descriptor_, buff, 3);
    if (size != 3)
    {
        perror("get_product_id");
        return -1;
    }
    if (tcdrain(file_descriptor_) < 0)
    {
        perror("get_product_id");
        return -1;
    }
    size = read(file_descriptor_, buff, 3);
    if (size != 3)
    {
        perror("get_product_id");
        return -1;
    }
    // Receiving data
    buff[0] = 0x04;
    buff[1] = 0x01;
    buff[2] = 0x01;
    size = write(file_descriptor_, buff, 3);
    if (size != 3)
    {
        perror("get_product_id");
        return -1;
    }
    if (tcdrain(file_descriptor_) < 0)
    {
        perror("get_product_id");
        return -1;
    }
    size = read(file_descriptor_, buff, 3);
    if (size != 3)
    {
        perror("get_product_id");
        return -1;
    }
    // Convert to short
    pid = big_endian_to_short(&buff[1]);
    return 0;

}

/**
 * @brief change big endian 2 byte into short
 * @param data Head pointer to the data
 * @retrun converted value
 */
int16_t DMU11::big_endian_to_short(char *data)
{
    char buff[2] = {data[1], data[0]};
    return *reinterpret_cast<int16_t *>(buff);
}

/**
 * @brief change big endian 4 byte into float
 * @param data Head pointer to the data
 * @retrun converted value
 */
float DMU11::big_endian_to_float(unsigned char *data)
{
    unsigned char buff[4] = {data[3], data[2], data[1], data[0]};
    return *reinterpret_cast<float *>(buff);
}


void DMU11::update()
{
    usleep(1000000);

    do
    {
        DMU11::dmu_package package;
        DMU11::bit32 input32;

//        unsigned char buff[68] = {0};
        char buff[68] = {0};

        usleep(100000);

        int size = read(file_descriptor_, buff, 68);
        if (size != 68)
        {
            std::cout << "size: " << size;
            perror("read failed");
            continue;
        }

        int16_t computed_checksum = 0;
        int16_t input16 = big_endian_to_short(&buff[0]);
        int16_t int16buff[33] = {0};


        if (input16 == header_)
        {
            package.header = input16;
            std::cout << "Header: " << std::hex << input16 << "\n";

            for (int i = 0; i < 64; i += 2)
            {
                int16buff[i / 2] = big_endian_to_short(&buff[i]);
                computed_checksum += int16buff[i / 2] ;
                std::cout << "Buffer [ " << i / 2 << " ]: " << int16buff[i / 2] << "\n";

            }
            computed_checksum = ~computed_checksum+0x0001;
            computed_checksum = computed_checksum&0xffff;
            int16_t checksum = big_endian_to_short(&buff[67]);
            std::cout << "Checksum: " << checksum << "\n";
            std::cout << "Computed Checksum: " << computed_checksum << "\n";
        }
        std::cout << "\n...end of 68 bytes package...\n";


    } while (terminate_flag_ == 0);

    //Converting to ROS format
    sensor_msgs::Imu raw_data;

    raw_data.header.frame_id = "imu";
    raw_data.header.stamp = ros::Time::now();

}





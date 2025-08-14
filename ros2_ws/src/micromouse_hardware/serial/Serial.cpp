#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>
#include <chrono>
#include <poll.h>

class RatzeHardwareInterface {
public:
    RatzeHardwareInterface() : nh_("~"), serial_fd_(-1), connected_(false), running_(false) {
        // Get parameters
        nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        nh_.param<int>("baudrate", baudrate_, 115200);
        
        // Initialize data structures
        tof_distances_.resize(7, 0);
        
        // Create publishers
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data", 10);
        heading_pub_ = nh_.advertise<std_msgs::Float32>("/imu/heading", 10);
        
        // Create ToF publishers
        tof_pubs_.resize(7);
        for (int i = 0; i < 7; i++) {
            std::string topic = "/tof/sensor" + std::to_string(i);
            tof_pubs_[i] = nh_.advertise<sensor_msgs::Range>(topic, 10);
        }
        
        encoder_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/encoder/counts", 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
        
        // Subscribe to commands
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &RatzeHardwareInterface::cmdVelCallback, this);
        
        // Setup serial connection
        if (!connect()) {
            ROS_ERROR("Failed to connect to serial port %s", port_.c_str());
        } else {
            ROS_INFO("Connected to %s at %d baud", port_.c_str(), baudrate_);
            // Calibrate heading on startup
            calibrateHeading();
        }
        
        // Setup timers
        timer_ = nh_.createTimer(ros::Duration(0.1), &RatzeHardwareInterface::timerCallback, this);
    }
    
    ~RatzeHardwareInterface() {
        disconnect();
    }

private:
    // ROS related
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    ros::Publisher heading_pub_;
    std::vector<ros::Publisher> tof_pubs_;
    ros::Publisher encoder_pub_;
    ros::Publisher odom_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Timer timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Serial communication
    int serial_fd_;
    std::string port_;
    int baudrate_;
    std::atomic<bool> connected_;
    std::thread reader_thread_;
    std::atomic<bool> running_;

    // Sensor data storage
    mutable std::mutex data_mutex_;
    float heading_ = 0.0;
    float roll_ = 0.0;
    float pitch_ = 0.0;
    float yaw_ = 0.0;
    int sys_calib_ = 0;
    int gyro_calib_ = 0;
    int accel_calib_ = 0;
    int mag_calib_ = 0;
    std::vector<uint16_t> tof_distances_;
    int32_t encoder1_ = 0;
    int32_t encoder2_ = 0;
    int32_t encoder3_ = 0;
    int32_t encoder4_ = 0;
    
    // Odometry calculation
    double x_ = 0.0, y_ = 0.0, odom_yaw_ = 0.0;
    double last_encoder_[4] = {0.0, 0.0, 0.0, 0.0};
    bool encoder_initialized_ = false;
    ros::Time last_odom_time_ = ros::Time::now();
    
    // Constants
    const double WHEEL_RADIUS = 0.035;  // 3.5cm radius
    const double TICKS_PER_REVOLUTION = 360.0;  // encoder ticks per wheel revolution
    const double WHEEL_BASE = 0.18;  // distance between wheels

    // Serial connection methods
    bool connect() {
        ROS_INFO("Connecting to %s at %d baud", port_.c_str(), baudrate_);
        
        // Open serial port
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            ROS_ERROR("Failed to open serial port: %s (%s)", port_.c_str(), strerror(errno));
            return false;
        }
        
        // Configure serial port
        struct termios options;
        tcgetattr(serial_fd_, &options);
        
        // Set baud rate
        speed_t baud;
        switch (baudrate_) {
            case 9600: baud = B9600; break;
            case 19200: baud = B19200; break;
            case 38400: baud = B38400; break;
            case 57600: baud = B57600; break;
            case 115200: baud = B115200; break;
            default:
                ROS_ERROR("Unsupported baud rate: %d", baudrate_);
                close(serial_fd_);
                return false;
        }
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        
        // 8N1, no flow control
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= CREAD | CLOCAL;
        
        // Raw input
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        options.c_oflag &= ~OPOST;
        
        // Apply settings
        tcsetattr(serial_fd_, TCSANOW, &options);
        
        // Set non-blocking
        fcntl(serial_fd_, F_SETFL, FNDELAY);
        
        // Wait for device to be ready
        tcflush(serial_fd_, TCIOFLUSH);
        
        // Wait for READY message
        char line[256];
        bool ready = false;
        auto start_time = std::chrono::steady_clock::now();
        
        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
            int n = readLine(serial_fd_, line, sizeof(line) - 1);
            if (n > 0) {
                line[n] = '\0';
                std::string s_line(line);
                ROS_INFO("Received: %s", s_line.c_str());
                if (s_line.find("READY") != std::string::npos) {
                    ready = true;
                    break;
                }
            }
            usleep(100000); // 100ms
        }
        
        if (!ready) {
            ROS_ERROR("Device not ready after 5 seconds");
            close(serial_fd_);
            return false;
        }
        
        connected_ = true;
        running_ = true;
        
        // Start reader thread
        reader_thread_ = std::thread(&RatzeHardwareInterface::readerThread, this);
        
        ROS_INFO("Connected successfully");
        return true;
    }

    void disconnect() {
        if (!connected_) return;
        
        // Stop reader thread
        running_ = false;
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
        
        // Close serial port
        if (serial_fd_ != -1) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        
        connected_ = false;
        ROS_INFO("Disconnected from %s", port_.c_str());
    }
    
    // Command methods
    bool moveForward(int speed) {
        return sendCommand('F', speed) && waitForAck('F');
    }

    bool moveBackward(int speed) {
        return sendCommand('B', speed) && waitForAck('B');
    }

    bool turnLeft(int speed) {
        return sendCommand('L', speed) && waitForAck('L');
    }

    bool turnRight(int speed) {
        return sendCommand('R', speed) && waitForAck('R');
    }

    bool stop() {
        return sendCommand('S') && waitForAck('S');
    }

    bool setSpeed(int speed) {
        return sendCommand('V', speed) && waitForAck('V');
    }

    bool resetEncoders() {
        return sendCommand('E') && waitForAck('E');
    }

    bool calibrateHeading() {
        return sendCommand('C') && waitForAck('C');
    }

    bool requestSensorData() {
        return sendCommand('G') && waitForAck('G');
    }
    
    bool sendCommand(char cmd, int value = 0) {
        if (!connected_) {
            ROS_ERROR("Not connected");
            return false;
        }
        
        std::string command;
        command += cmd;
        if (value > 0) {
            command += std::to_string(value);
        }
        command += '\n';
        
        ssize_t bytes_written = write(serial_fd_, command.c_str(), command.length());
        if (bytes_written != static_cast<ssize_t>(command.length())) {
            ROS_ERROR("Failed to send command: %s", command.c_str());
            return false;
        }
        
        return true;
    }

    bool waitForAck(char cmd, int timeout_ms = 1000) {
        if (!connected_) return false;
        
        std::string expected_ack = std::string("ACK:") + cmd;
        char line_buffer[256];
        auto start_time = std::chrono::steady_clock::now();
        
        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(timeout_ms)) {
            int n = readLine(serial_fd_, line_buffer, sizeof(line_buffer) - 1);
            if (n > 0) {
                line_buffer[n] = '\0';
                std::string line(line_buffer);
                
                if (line.find(expected_ack) != std::string::npos) {
                    return true;
                }
                else if (line.find("ERR:") != std::string::npos) {
                    ROS_ERROR("Command error: %s", line.c_str());
                    return false;
                }
            }
            
            usleep(10000); // 10ms
        }
        
        ROS_ERROR("Timeout waiting for acknowledgment: %s", expected_ack.c_str());
        return false;
    }

    int readLine(int fd, char* buffer, int max_size) {
        int n = 0;
        char c;
        
        struct pollfd fds[1];
        fds[0].fd = fd;
        fds[0].events = POLLIN;
        
        while (n < max_size) {
            // Poll with timeout
            int poll_result = poll(fds, 1, 100); // 100ms timeout
            
            if (poll_result <= 0) {
                // Timeout or error
                return n;
            }
            
            // Data available
            if (read(fd, &c, 1) == 1) {
                buffer[n++] = c;
                if (c == '\n') {
                    break;
                }
            } else {
                // Error or no more data
                break;
            }
        }
        
        return n;
    }
    
    // Reader thread method
    void readerThread() {
        char line_buffer[256];
        
        while (running_ && connected_) {
            int n = readLine(serial_fd_, line_buffer, sizeof(line_buffer) - 1);
            
            if (n > 0) {
                line_buffer[n] = '\0';
                std::string line(line_buffer);
                
                // Parse data based on prefix
                if (line.find("IMU,") == 0) {
                    parseImuData(line);
                }
                else if (line.find("TOF,") == 0) {
                    parseTofData(line);
                }
                else if (line.find("ENC,") == 0) {
                    parseEncoderData(line);
                }
                else if (line.find("ACK:") == 0) {
                    // Acknowledgment received - handled in waitForAck
                }
                else if (line.find("ERR:") == 0) {
                    ROS_ERROR("Error from device: %s", line.c_str());
                }
                else if (line.find("#") == 0) {
                    ROS_INFO("Device message: %s", line.c_str());
                }
            }
            
            usleep(10000); // 10ms
        }
    }
    
    // Data parsing methods
    bool parseImuData(const std::string& line) {
        // Format: IMU,heading,roll,pitch,yaw,sys,gyro,accel,mag
        std::istringstream iss(line.substr(4)); // Skip "IMU,"
        std::string token;
        std::vector<float> values;
        std::vector<int> calib;
        
        // Parse floating point values
        for (int i = 0; i < 4; i++) {
            if (!std::getline(iss, token, ',')) {
                ROS_ERROR("Failed to parse IMU data: %s", line.c_str());
                return false;
            }
            try {
                values.push_back(std::stof(token));
            } catch (...) {
                ROS_ERROR("Failed to parse IMU value: %s", token.c_str());
                return false;
            }
        }
        
        // Parse calibration integers
        for (int i = 0; i < 4; i++) {
            if (!std::getline(iss, token, ',')) {
                if (i == 3) break; // Last value might not have comma
                ROS_ERROR("Failed to parse IMU calibration: %s", line.c_str());
                return false;
            }
            try {
                calib.push_back(std::stoi(token));
            } catch (...) {
                ROS_ERROR("Failed to parse IMU calibration value: %s", token.c_str());
                return false;
            }
        }
        
        if (values.size() < 4 || calib.size() < 4) {
            ROS_ERROR("Incomplete IMU data: %s", line.c_str());
            return false;
        }
        
        // Store data and publish
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            heading_ = values[0];
            roll_ = values[1];
            pitch_ = values[2];
            yaw_ = values[3];
            sys_calib_ = calib[0];
            gyro_calib_ = calib[1];
            accel_calib_ = calib[2];
            mag_calib_ = calib[3];
        }
        
        // Publish IMU data
        publishImuData();
        
        return true;
    }

    bool parseTofData(const std::string& line) {
        // Format: TOF,s0,s1,s2,s3,s4,s5,s6
        std::istringstream iss(line.substr(4)); // Skip "TOF,"
        std::string token;
        std::vector<uint16_t> values;
        
        while (std::getline(iss, token, ',')) {
            try {
                values.push_back(static_cast<uint16_t>(std::stoi(token)));
            } catch (...) {
                ROS_ERROR("Failed to parse ToF data: %s", line.c_str());
                return false;
            }
        }
        
        if (values.size() < tof_distances_.size()) {
            ROS_ERROR("Incomplete ToF data: %s", line.c_str());
            return false;
        }
        
        // Store data and publish
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (size_t i = 0; i < tof_distances_.size(); ++i) {
                if (i < values.size()) {
                    tof_distances_[i] = values[i];
                }
            }
        }
        
        // Publish ToF data
        publishTofData();
        
        return true;
    }

    bool parseEncoderData(const std::string& line) {
        // Format: ENC,m1,m2,m3,m4
        std::istringstream iss(line.substr(4)); // Skip "ENC,"
        std::string token;
        std::vector<int32_t> values;
        
        while (std::getline(iss, token, ',')) {
            try {
                values.push_back(std::stoi(token));
            } catch (...) {
                ROS_ERROR("Failed to parse encoder data: %s", line.c_str());
                return false;
            }
        }
        
        if (values.size() < 4) {
            ROS_ERROR("Incomplete encoder data: %s", line.c_str());
            return false;
        }
        
        // Store data and publish
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            encoder1_ = values[0];
            encoder2_ = values[1];
            encoder3_ = values[2];
            encoder4_ = values[3];
        }
        
        // Publish encoder data and update odometry
        publishEncoderData();
        
        return true;
    }
    
    // ROS publishing methods
    void publishImuData() {
        // Make a local copy to avoid holding the mutex during publishing
        float heading, roll, pitch, yaw;
        int sys_calib, gyro_calib, accel_calib, mag_calib;
        
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            heading = heading_;
            roll = roll_;
            pitch = pitch_;
            yaw = yaw_;
            sys_calib = sys_calib_;
            gyro_calib = gyro_calib_;
            accel_calib = accel_calib_;
            mag_calib = mag_calib_;
        }
        
        // Publish heading as a Float32
        std_msgs::Float32 heading_msg;
        heading_msg.data = heading;
        heading_pub_.publish(heading_msg);
        
        // Create IMU message
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";
        
        // Convert heading to quaternion (assuming level orientation)
        double heading_rad = heading * M_PI / 180.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, heading_rad);
        
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
        
        // We don't have angular velocity or linear acceleration data
        // in the heading-only mode, so these are left as zeros
        
        // Set covariance based on calibration
        double orientation_covariance = 0.1;
        if (mag_calib == 3 && sys_calib == 3) {
            orientation_covariance = 0.01;
        }
        
        for (int i = 0; i < 9; ++i) {
            imu_msg.orientation_covariance[i] = (i == 0 || i == 4 || i == 8) ? orientation_covariance : 0;
        }
        
        // Publish
        imu_pub_.publish(imu_msg);
        
        // Update yaw for odometry
        odom_yaw_ = heading_rad;
    }
    
    void publishTofData() {
        // Make a local copy
        std::vector<uint16_t> distances;
        
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            distances = tof_distances_;
        }
        
        sensor_msgs::Range range_msg;
        range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        range_msg.field_of_view = 0.44;  // ~25 degrees in radians
        range_msg.min_range = 0.05;      // 5cm
        range_msg.max_range = 2.0;       // 2m
        
        for (size_t i = 0; i < distances.size() && i < tof_pubs_.size(); ++i) {
            range_msg.header.stamp = ros::Time::now();
            range_msg.header.frame_id = "tof_" + std::to_string(i);
            range_msg.range = distances[i] / 1000.0;  // convert mm to m
            tof_pubs_[i].publish(range_msg);
        }
    }
    
    void publishEncoderData() {
        // Make a local copy
        int32_t e1, e2, e3, e4;
        
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            e1 = encoder1_;
            e2 = encoder2_;
            e3 = encoder3_;
            e4 = encoder4_;
        }
        
        // Publish raw encoder counts
        std_msgs::Int32MultiArray encoder_msg;
        encoder_msg.data.push_back(e1);
        encoder_msg.data.push_back(e2);
        encoder_msg.data.push_back(e3);
        encoder_msg.data.push_back(e4);
        encoder_pub_.publish(encoder_msg);
        
        // Update odometry
        ros::Time current_time = ros::Time::now();
        
        if (!encoder_initialized_) {
            last_encoder_[0] = e1;
            last_encoder_[1] = e2;
            last_encoder_[2] = e3;
            last_encoder_[3] = e4;
            encoder_initialized_ = true;
            last_odom_time_ = current_time;
            return;
        }
        
        // Calculate wheel rotations
        double delta_enc[4];
        delta_enc[0] = e1 - last_encoder_[0];
        delta_enc[1] = e2 - last_encoder_[1];
        delta_enc[2] = e3 - last_encoder_[2];
        delta_enc[3] = e4 - last_encoder_[3];
        
        // Save current encoder values
        last_encoder_[0] = e1;
        last_encoder_[1] = e2;
        last_encoder_[2] = e3;
        last_encoder_[3] = e4;
        
        // Calculate average rotation for each side (left: 0,2; right: 1,3)
        double left_ticks = (delta_enc[0] + delta_enc[2]) / 2.0;
        double right_ticks = (delta_enc[1] + delta_enc[3]) / 2.0;
        
        // Convert to distance
        double left_distance = (left_ticks / TICKS_PER_REVOLUTION) * 2 * M_PI * WHEEL_RADIUS;
        double right_distance = (right_ticks / TICKS_PER_REVOLUTION) * 2 * M_PI * WHEEL_RADIUS;
        
        // Calculate robot motion
        double distance = (left_distance + right_distance) / 2.0;
        
        // Update position using heading from IMU
        x_ += distance * cos(odom_yaw_);
        y_ += distance * sin(odom_yaw_);
        
        // Create and publish odometry message
        publishOdometry(current_time, distance);
    }
    
    void publishOdometry(const ros::Time& current_time, double distance) {
        // Create quaternion from yaw
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_yaw_);
        
        // Publish transform
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_.sendTransform(transform);
        
        // Publish odometry message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // Set position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // Calculate velocity
        double dt = (current_time - last_odom_time_).toSec();
        if (dt > 0) {
            double linear_vel = distance / dt;
            double angular_vel = 0.0; // We're using IMU for heading, so no angular velocity from encoders
            
            odom_msg.twist.twist.linear.x = linear_vel;
            odom_msg.twist.twist.angular.z = angular_vel;
            
            // Set covariances
            for (int i = 0; i < 36; ++i) {
                odom_msg.pose.covariance[i] = 0.0;
                odom_msg.twist.covariance[i] = 0.0;
            }
            
            // Position covariance
            odom_msg.pose.covariance[0] = 0.01;  // x
            odom_msg.pose.covariance[7] = 0.01;  // y
            odom_msg.pose.covariance[35] = 0.02; // yaw
            
            // Velocity covariance
            odom_msg.twist.covariance[0] = 0.01;  // x velocity
            odom_msg.twist.covariance[35] = 0.01; // yaw velocity
        }
        
        // Publish odometry
        odom_pub_.publish(odom_msg);
        
        last_odom_time_ = current_time;
    }
    
    // ROS callback methods
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // Convert twist to motor commands
        double linear = msg->linear.x;
        double angular = msg->angular.z;
        
        if (fabs(linear) < 0.01 && fabs(angular) < 0.01) {
            stop();
            return;
        }
        
        const int MAX_SPEED = 255;
        int speed = static_cast<int>(fabs(linear) * MAX_SPEED);
        
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        if (speed < 0) speed = 0;
        
        // Simple differential drive control
        if (fabs(angular) > 0.1) {
            if (angular > 0) {
                turnLeft(speed);
            } else {
                turnRight(speed);
            }
        } else if (linear > 0) {
            moveForward(speed);
        } else if (linear < 0) {
            moveBackward(speed);
        }
    }
    
    void timerCallback(const ros::TimerEvent&) {
        // Request sensor data periodically
        if (connected_) {
            requestSensorData();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ratze_hardware_interface");
    
    RatzeHardwareInterface interface;
    
    ros::spin();
    
    return 0;
}
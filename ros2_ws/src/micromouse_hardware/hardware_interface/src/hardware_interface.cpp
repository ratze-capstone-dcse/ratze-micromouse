#include <hardware_interface/hardware_interface.hpp>

using namespace std::chrono_literals;

namespace ratze_hardware_interface
{

    RatzeHardwareInterface::RatzeHardwareInterface() : Node("ratze_hardware_interface"), connected_(false), running_(false),
                                                       last_odom_time_(this->now())
    {
        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);

        // Get parameters
        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();

        // Initialize data structures
        tof_distances_.resize(7, 0);

        // Create publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu/heading", 10);

        // Create ToF publishers
        tof_pubs_.resize(7);
        for (int i = 0; i < 7; i++)
        {
            std::string topic = "tof/sensor" + std::to_string(i);
            tof_pubs_[i] = this->create_publisher<sensor_msgs::msg::Range>(topic, 10);
        }

        // Add LaserScan publisher
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("encoder/counts", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Create subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&RatzeHardwareInterface::cmdVelCallback, this, std::placeholders::_1));
        user_command_ = this->create_subscription<std_msgs::msg::Char>(
            "user_command", 10,
            std::bind(&RatzeHardwareInterface::userCommandCallback, this, std::placeholders::_1));

        // Create tf broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // create later transform broadcaster
        laser_transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize SerialPort object
        serial_port_ = std::make_unique<LibSerial::SerialPort>();

        // Setup serial connection
        if (!connect())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to serial port %s", port_.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Connected to %s at %d baud", port_.c_str(), baudrate_);
            // Calibrate heading on startup
            calibrateHeading();
        }

        // Setup timers
        timer_ = this->create_wall_timer(
            100ms, std::bind(&RatzeHardwareInterface::timerCallback, this));
    }

    RatzeHardwareInterface::~RatzeHardwareInterface()
    {
        disconnect();
    }

    bool RatzeHardwareInterface::connect()
    {
        RCLCPP_INFO(this->get_logger(), "Connecting to %s at %d baud", port_.c_str(), baudrate_);

        try
        {
            // open serial port
            serial_port_->Open(port_);

            // configure serial port
            serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial_port_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_->SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_port_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

            // flush data in serial buffer
            serial_port_->FlushIOBuffers();

            connected_ = true;
            running_ = true;

            // Start the serial read thread
            read_thread_ = std::thread(&RatzeHardwareInterface::readThread, this);

            RCLCPP_INFO(this->get_logger(), "Successfully connected to %s", port_.c_str());
            return true;
        }
        catch (const LibSerial::OpenFailed &)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", ex.what());
        }
        return false;
    }
    void RatzeHardwareInterface::disconnect()
    {
        if (!connected_)
            return;
        // stop reader thread
        running_ = false;
        if (read_thread_.joinable())
        {
            read_thread_.join();
        }
        // close serial port
        try
        {
            if (serial_port_ && serial_port_->IsOpen())
            {
                serial_port_->Close();
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to close serial port: %s", ex.what());
        }
        connected_ = false;
        RCLCPP_INFO(this->get_logger(), "Disconnected from %s", port_.c_str());
    }

    void RatzeHardwareInterface::userCommandCallback(const std_msgs::msg::Char::SharedPtr msg)
    {
        try {
            serial_port_->Write(std::string(1, msg->data));
            RCLCPP_INFO(this->get_logger(), "Received user command: %c", msg->data);
        }
        catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send user command: %s", ex.what());
        }
    }

    bool RatzeHardwareInterface::sendCommand(char cmd, int value)
    {
        if (!connected_ || !serial_port_->IsOpen())
        {
            RCLCPP_ERROR(this->get_logger(), "Not connected");
            return false;
        }
        std::string command;
        command += cmd;
        if (value > 0)
        {
            command += std::to_string(value);
        }
        command += '\n';
        try
        {
            serial_port_->Write(command);
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s - %s", command.c_str(), e.what());
            return false;
        }
    }

    bool RatzeHardwareInterface::waitForAck(char cmd, int timeout_ms)
    {
        if (!connected_)
            return false;

        std::string expected_ack = std::string("ACK:") + cmd;
        auto start_time = std::chrono::steady_clock::now();

        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(timeout_ms))
        {
            std::string line = readLine(500);

            if (!line.empty())
            {
                if (line.find(expected_ack) != std::string::npos)
                {
                    return true;
                }
                else if (line.find("ERR:") != std::string::npos)
                {
                    RCLCPP_ERROR(this->get_logger(), "Command error: %s", line.c_str());
                    return false;
                }
            }

            std::this_thread::sleep_for(10ms);
        }

        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for acknowledgment: %s", expected_ack.c_str());
        return false;
    }

    std::string RatzeHardwareInterface::readLine(int timeout_ms)
    {
        if (!connected_ || !serial_port_->IsOpen())
        {
            return "";
        }

        std::string line;
        try
        {
            // Set timeout first
            serial_port_->SetVTime(timeout_ms / 100); // VTime is in deciseconds

            // Then read a line with proper arguments
            serial_port_->ReadLine(line, '\n');

            // Trim CR/LF
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

            return line;
        }
        catch (const LibSerial::ReadTimeout &)
        {
            return "";
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            return "";
        }
    }

    void RatzeHardwareInterface::readThread()
    {
        while (running_ && connected_)
        {
            std::string line = readLine();

            if (!line.empty())
            {
                // Parse data based on prefix
                if (line.find("IMU,") == 0)
                {
                    parseImuData(line);
                }
                else if (line.find("TOF,") == 0)
                {
                    parseTofData(line);
                }
                else if (line.find("ENC,") == 0)
                {
                    parseEncoderData(line);
                }
                else if (line.find("ACK:") == 0)
                {
                    // Acknowledgment received - handled in waitForAck
                }
                else if (line.find("ERR:") == 0)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error from device: %s", line.c_str());
                }
                else if (line.find("#") == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Device message: %s", line.c_str());
                }
            }

            std::this_thread::sleep_for(10ms);
        }
    }

    bool RatzeHardwareInterface::parseImuData(const std::string &line)
    {
        // Format: IMU,roll,pitch,yaw,gyro_x,gyro_y,gyro_z,accel_x,accell_y,accell_z
        std::istringstream iss(line.substr(4)); // Skip "IMU,"
        std::string token;
        std::vector<float> values;

        // Parse floating point values
        for (int i = 0; i < 9; i++)
        {
            if (!std::getline(iss, token, ','))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse IMU data: %s", line.c_str());
                return false;
            }
            try
            {
                values.push_back(std::stof(token));
            }
            catch (...)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse IMU value: %s", token.c_str());
                return false;
            }
        }

        // Store data and publish
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            roll_ = values[0];
            pitch_ = values[1];
            yaw_ = values[2];
            gyro_x_ = values[3];
            gyro_y_ = values[4];
            gyro_z_ = values[5];
            accel_x_ = values[6];
            accel_y_ = values[7];
            accel_z_ = values[8];
        }

        // Publish IMU data
        publishImuData();

        return true;
    }

    void RatzeHardwareInterface::publishImuData()
    {
        // Make a local copy to avoid holding the mutex during publishing
        float heading, roll, pitch, yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
        int sys_calib, gyro_calib, accel_calib, mag_calib;


        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            roll = roll_;
            pitch = pitch_;
            yaw = yaw_;
            gyro_x = gyro_x_;
            gyro_y = gyro_y_;
            gyro_z = gyro_z_;
            accel_x = accel_x_;
            accel_y = accel_y_;
            accel_z = accel_z_;
        }

        // Publish heading as a Float32
        auto heading_msg = std_msgs::msg::Float32();
        heading_msg.data = yaw; // Use yaw as heading
        heading_pub_->publish(heading_msg);

        // Create IMU message
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "base_link";

        // Convert roll, pitch, yaw to quaternion
        // double heading_rad = yaw * M_PI / 180.0;
        // double roll_rad = roll * M_PI / 180.0;
        // double pitch_rad = pitch * M_PI / 180.0;
        
        tf2::Quaternion q;
        q.setRPY(roll, pitch, heading);
        q.normalize();


        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
        
        // Add angular velocity data
        imu_msg.angular_velocity.x = gyro_x;
        imu_msg.angular_velocity.y = gyro_y;
        imu_msg.angular_velocity.z = gyro_z;
        
        // Add linear acceleration data
        imu_msg.linear_acceleration.x = accel_x;
        imu_msg.linear_acceleration.y = accel_y;
        imu_msg.linear_acceleration.z = accel_z;

        // We don't have angular velocity or linear acceleration data
        // in the heading-only mode, so these are left as zeros

        // Set covariance based on calibration
        double orientation_covariance = 0.1;
        if (mag_calib == 3 && sys_calib == 3)
        {
            orientation_covariance = 0.01;
        }

        for (int i = 0; i < 9; ++i)
        {
            imu_msg.orientation_covariance[i] = (i == 0 || i == 4 || i == 8) ? orientation_covariance : 0;
        }

        // Publish
        imu_pub_->publish(imu_msg);

        // Update yaw for odometry
        odom_yaw_ = heading_rad;
    }

    bool RatzeHardwareInterface::parseTofData(const std::string &line)
    {
        // Format: TOF,s0,s1,s2,s3,s4,s5,s6
        std::istringstream iss(line.substr(4)); // Skip "TOF,"
        std::string token;
        std::vector<uint16_t> values;

        while (std::getline(iss, token, ','))
        {
            try
            {
                values.push_back(static_cast<uint16_t>(std::stoi(token)));
            }
            catch (...)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse ToF data: %s", line.c_str());
                return false;
            }
        }

        if (values.size() < tof_distances_.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Incomplete ToF data: %s", line.c_str());
            return false;
        }

        // Store data and publish
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (size_t i = 0; i < tof_distances_.size(); ++i)
            {
                if (i < values.size())
                {
                    tof_distances_[i] = values[i];
                }
            }
        }

        // Publish ToF data
        publishTofData();

        return true;
    }

    void RatzeHardwareInterface::publishTofData()
    {
        // Make a local copy
        std::vector<uint16_t> distances;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            distances = tof_distances_;
        }

        auto range_msg = sensor_msgs::msg::Range();
        range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        range_msg.field_of_view = 0.44; // ~25 degrees in radians
        range_msg.min_range = 0.05;     // 5cm
        range_msg.max_range = 2.0;      // 2m

        std::stringstream ss;

        for (size_t i = 0; i < distances.size() && i < tof_pubs_.size(); ++i)
        {
            range_msg.header.stamp = this->now();
            range_msg.header.frame_id = "tof_" + std::to_string(i);
            range_msg.range = distances[i] / 1000.0; // convert mm to m

            // Individual log for each sensor
            RCLCPP_INFO(this->get_logger(), "ToF sensor %zu: %.3f m", i, range_msg.range);
            tof_pubs_[i]->publish(range_msg);
        }

        // Publish as LaserScan
        publishLaserScan(distances);
    }

    void RatzeHardwareInterface::publishLaserScan(const std::vector<uint16_t> &distances)
    {
        auto scan_msg = sensor_msgs::msg::LaserScan();
        scan_msg.header.stamp = this->now();
        scan_msg.header.frame_id = "ray_5_link";

        // Configure scan parameters
        scan_msg.angle_min = -M_PI / 2.0;  // -90 degrees
        scan_msg.angle_max = M_PI / 2.0;   // 90 degrees

        // 7 sensors across 180 degrees
        scan_msg.angle_increment = M_PI / 6.0;  // 30 degrees

        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 0.1;  // 10Hz scan rate

        scan_msg.range_min = 0.01;  // 1cm
        scan_msg.range_max = 2.0;   // 2m

        // Initialize ranges array with inf values
        scan_msg.ranges.resize(7, std::numeric_limits<float>::infinity());

        float range_meters = distances[2] / 1000.0; // mm to meters
        if (range_meters < scan_msg.range_min || range_meters > scan_msg.range_max)
            scan_msg.ranges[0] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[0] = range_meters;

        range_meters = distances[3] / 1000.0; // mm to meters
        if (range_meters < scan_msg.range_min || range_meters > scan_msg.range_max)
            scan_msg.ranges[1] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[1] = range_meters;
        

        range_meters = distances[4] / 1000.0; // mm to meters
        if (range_meters < scan_msg.range_min || range_meters > scan_msg.range_max)
            scan_msg.ranges[2] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[2] = range_meters;

        range_meters = distances[5] / 1000.0; // mm to meters
        if (range_meters < scan_msg.range_min || range_meters > scan_msg.range_max)
            scan_msg.ranges[3] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[3] = range_meters;
        

        range_meters = distances[6] / 1000.0; // mm to meters
        if (range_meters < scan_msg.range_min || range_meters > scan_msg.range_max)
            scan_msg.ranges[4] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[4] = range_meters;
        

        range_meters = distances[1] / 1000.0; // mm to meters
        if (range_meters < scan_msg.range_min || range_meters > scan_msg.range_max)
            scan_msg.ranges[5] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[5] = range_meters;
        

        range_meters = distances[0] / 1000.0; // mm to meters
        if (range_meters < scan_msg.range_min || range_meters > scan_msg.range_max)
            scan_msg.ranges[6] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[6] = range_meters;
        

        // Publish the scan
        laser_pub_->publish(scan_msg);
    }

    void RatzeHardwareInterface::publishLaserTransform(){
        // publish transform for laser scanner
        auto transform = geometry_msgs::msg::TransformStamped();
        transform.header.stamp = this->now();
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "ray_5_link";
        transform.transform.translation.x = 0.1; // adjust as needed
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.05; // adjust as needed
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        laser_transform_broadcaster_->sendTransform(transform);
    }

    bool RatzeHardwareInterface::parseEncoderData(const std::string &line)
    {
        // Format: ENC,m1,m2,m3,m4
        std::istringstream iss(line.substr(4)); // Skip "ENC,"
        std::string token;
        std::vector<int32_t> values;

        while (std::getline(iss, token, ','))
        {
            try
            {
                values.push_back(std::stoi(token));
            }
            catch (...)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse encoder data: %s", line.c_str());
                return false;
            }
        }

        if (values.size() < 4)
        {
            RCLCPP_ERROR(this->get_logger(), "Incomplete encoder data: %s", line.c_str());
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

    void RatzeHardwareInterface::publishEncoderData()
    {
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
        auto encoder_msg = std_msgs::msg::Int32MultiArray();
        encoder_msg.data = {e1, e2, e3, e4};

        RCLCPP_INFO(this->get_logger(), "Publishing Encoders: %d, %d, %d, %d", e1, e2, e3, e4);

        encoder_pub_->publish(encoder_msg);

        // Update odometry
        rclcpp::Time current_time = this->now();

        if (!encoder_initialized_)
        {
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
        double right_ticks = (delta_enc[0] + delta_enc[2]) / 2.0;
        double left_ticks = (delta_enc[1] + delta_enc[3]) / 2.0;

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

    void RatzeHardwareInterface::publishOdometry(const rclcpp::Time &current_time, double distance)
    {
        // create quaternion from yaw
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_yaw_);

        // publish transform
        auto transform = geometry_msgs::msg::TransformStamped();
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

        tf_broadcaster_->sendTransform(transform);

        // publish odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // set position
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // calculate velocity
        double dt = (current_time - last_odom_time_).seconds();
        if (dt > 0)
        {
            odom_msg.twist.twist.linear.x = distance / dt; // linear velocity
            odom_msg.twist.twist.linear.y = 0.0;           // angular velocity not calculated, using imu for heading

            // set covariance
            for (int i = 0; i < 36; ++i)
            {
                odom_msg.pose.covariance[i] = 0.0;
                odom_msg.twist.covariance[i] = 0.0;
            }

            // position covariance
            odom_msg.pose.covariance[0] = 0.01;  // x
            odom_msg.pose.covariance[7] = 0.01;  // y
            odom_msg.pose.covariance[35] = 0.02; // z

            // velocity covariance
            odom_msg.twist.covariance[0] = 0.01;  // linear x
            odom_msg.twist.covariance[35] = 0.01; // angular z
        }
        // publish odometry message
        odom_pub_->publish(odom_msg);

        last_odom_time_ = current_time;
    }
    void RatzeHardwareInterface::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // convert twist to motor commands
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        if (fabs(linear) < 0.01 && fabs(angular) < 0.01)
        {
            stop();
            return;
        }

        const int MAX_SPEED = 255;
        int speed = static_cast<int>(fabs(linear) * MAX_SPEED);
        if (speed > MAX_SPEED)
        {
            speed = MAX_SPEED;
        }
        // Simple differential drive control
        if (fabs(angular) > 0.1)
        {
            if (angular > 0)
            {
                turnLeft(speed);
            }
            else
            {
                turnRight(speed);
            }
        }
        else if (linear > 0)
        {
            moveForward(speed);
        }
        else if (linear < 0)
        {
            moveBackward(speed);
        }
    }

    bool RatzeHardwareInterface::moveForward(int speed)
    {
        return sendCommand('F', speed) && waitForAck('F');
    }

    bool RatzeHardwareInterface::moveBackward(int speed)
    {
        return sendCommand('B', speed) && waitForAck('B');
    }

    bool RatzeHardwareInterface::turnLeft(int speed)
    {
        return sendCommand('L', speed) && waitForAck('L');
    }

    bool RatzeHardwareInterface::turnRight(int speed)
    {
        return sendCommand('R', speed) && waitForAck('R');
    }

    bool RatzeHardwareInterface::stop()
    {
        return sendCommand('S') && waitForAck('S');
    }

    bool RatzeHardwareInterface::setSpeed(int speed)
    {
        return sendCommand('V', speed) && waitForAck('V');
    }

    bool RatzeHardwareInterface::resetEncoders()
    {
        return sendCommand('E') && waitForAck('E');
    }

    bool RatzeHardwareInterface::calibrateHeading()
    {
        return sendCommand('C') && waitForAck('C');
    }

    bool RatzeHardwareInterface::requestSensorData()
    {
        return sendCommand('G') && waitForAck('G');
    }

    void RatzeHardwareInterface::timerCallback()
    {
        static rclcpp::Time last_scan_time = this->now();
        rclcpp::Time current_time = this->now();
        // Periodically request sensor data
        if (connected_)
        {
            // requestSensorData();

            publishLaserTransform();

            // Publish sensor data
            publishImuData();
            // publishTofData();
            publishEncoderData();

            if ((current_time - last_scan_time).seconds() >= 0.1)
            {
                publishTofData();
                last_scan_time = current_time;
            }
        }
    }
} // namespace micromouse_hardware

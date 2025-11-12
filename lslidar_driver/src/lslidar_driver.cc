/*
 * This file is part of lslidar driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <arpa/inet.h>
#include <cmath>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

#include "lslidar_driver/lslidar_driver.h"
#include "rclcpp/rclcpp.hpp"
#include <functional>

int truncated_mode_ = 0; // 多角度屏蔽开关：默认为0，如果需要屏蔽多个角度，则truncated_mode_赋值为1。

int scan_crop_min[] = { 0, 180 }; // 雷达屏蔽角度，这里屏蔽角度为135°到225°，
                                  // 如果要多角度屏蔽，如10~30，50~60，改为：
                                  // scan_angle_min[]={10，50};scan_angle_max[]={30，60};
int scan_crop_max[] = { 90, 270 }; // 修改后编译即可

namespace lslidar_driver {

LslidarDriver::LslidarDriver() : LslidarDriver(rclcpp::NodeOptions()) { }
LslidarDriver::LslidarDriver(const rclcpp::NodeOptions& options)
    : Node("lslidar_driver_node", options)
    , diagnostics_(this)
    , out_stream_("lidar_raw.bin", std::ios::binary) {

    if (!this->initialize()) RCLCPP_ERROR(this->get_logger(), "Could not initialize the driver...");
    else RCLCPP_INFO(this->get_logger(), "Successfully initialize driver...");
}

LslidarDriver::~LslidarDriver() {
    out_stream_.close();
    return;
}

bool LslidarDriver::loadParameters() {
    pubscan_thread_ = new boost::thread(boost::bind(&LslidarDriver::pubScanThread, this));
    serial_read_buf_.reserve(200);
    // interface_selection_ = std::string("net");
    // frame_id_ = std::string("laser_link");
    // scan_topic_ = std::string("/scan");
    // lidar_name_ = std::string("M10");
    // pointcloud_topic_ = std::string("/lslidar_point_cloud");
    is_start_ = true;
    // min_range_ = 0.3;
    // max_range_ = 100.0;
    // use_gps_ts_ = true;
    // compensation_ = true;
    // pubScan_ = true;
    // pubPointCloud2_ = true;
    // angle_disable_min_ = 0.0;
    // angle_disable_max_ = 0.0;

    this->declare_parameter<std::string>("lidar_name", "M10_P");
    this->declare_parameter<std::string>("frame_id", "lidar_link");
    this->declare_parameter<std::string>("scan_topic", "/scan_raw");
    this->declare_parameter<std::string>("pointcloud_topic", "/lslidar_point_cloud");
    this->declare_parameter<double>("min_range", 0.3);
    this->declare_parameter<double>("max_range", 100.0);
    this->declare_parameter<bool>("use_gps_ts", false);
    this->declare_parameter<bool>("high_reflection", false);
    this->declare_parameter<bool>("compensation", false);
    this->declare_parameter<bool>("pubScan", true);
    this->declare_parameter<bool>("pubPointCloud2", false);
    this->declare_parameter<double>("angle_disable_min", 0.0);
    this->declare_parameter<double>("angle_disable_max", 0.0);
    this->declare_parameter<std::string>("interface_selection", "serial");
    this->declare_parameter<std::string>("serial_port_", "/dev/ttyACM2");
    this->declare_parameter<int>("max_consecutive_failed_reads", 150); // 0.5s straight of failed reads

    this->get_parameter("lidar_name", lidar_name_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("high_reflection", high_reflection_);
    this->get_parameter("scan_topic", scan_topic_);
    this->get_parameter("min_range", min_range_);
    this->get_parameter("max_range", max_range_);
    this->get_parameter("use_gps_ts", use_gps_ts_);
    this->get_parameter("compensation", compensation_);
    this->get_parameter("pointcloud_topic", pointcloud_topic_);
    this->get_parameter("pubScan", pubScan_);
    this->get_parameter("pubPointCloud2", pubPointCloud2_);
    this->get_parameter("angle_disable_min", angle_disable_min_);
    this->get_parameter("angle_disable_max", angle_disable_max_);
    this->get_parameter("interface_selection", interface_selection_);
    this->get_parameter("max_consecutive_failed_reads", max_consecutive_failed_reads_);
    while (angle_disable_min_ < 0)
        angle_disable_min_ += 360;
    while (angle_disable_max_ < 0)
        angle_disable_max_ += 360;
    while (angle_disable_min_ > 360)
        angle_disable_min_ -= 360;
    while (angle_disable_max_ > 360)
        angle_disable_max_ -= 360;
    if (angle_disable_max_ == angle_disable_min_) {
        angle_able_min_ = 0;
        angle_able_max_ = 360;
    } else {
        if (angle_disable_min_ < angle_disable_max_ && angle_disable_min_ != 0.0) {
            angle_able_min_ = angle_disable_max_;
            angle_able_max_ = angle_disable_min_ + 360;
        }
        if (angle_disable_min_ < angle_disable_max_ && angle_disable_min_ == 0.0) {
            angle_able_min_ = angle_disable_max_;
            angle_able_max_ = 360;
        }
        if (angle_disable_min_ > angle_disable_max_) {
            angle_able_min_ = angle_disable_max_;
            angle_able_max_ = angle_disable_min_;
        }
    }
    pub_sample_count_shared_ = 0;

    /*
    if (lidar_name == "M10") {
        use_gps_ts = false;
        PACKET_SIZE = 92;
        package_points_ = 42;
        data_bits_start_ = 6;
        degree_bits_start_ = 2;
        rpm_bits_start_ = 4;
        baud_rate_ = 460800;
        points_size_ = 1008;
    } else if (lidar_name == "M10_PLUS") {
        PACKET_SIZE = 104;
        package_points_ = 41;
        data_bits_start_ = 8;
        degree_bits_start_ = 4;
        rpm_bits_start_ = 6;
        points_size_ = 5000;
        baud_rate_ = 921600;
    } else if (lidar_name == "M10_GPS") {
        PACKET_SIZE = 102;
        package_points_ = 42;
        data_bits_start_ = 6;
        degree_bits_start_ = 2;
        rpm_bits_start_ = 4;
        baud_rate_ = 460800;
        points_size_ = 1008;
    } else if (lidar_name == "N10") {
        PACKET_SIZE = 58;
        package_points_ = 16;
        data_bits_start_ = 7;
        degree_bits_start_ = 5;
        end_degree_bits_start_ = 55;
        baud_rate_ = 230400;
        points_size_ = 2000;
        use_gps_ts = false;
        compensation = false;
    } else if (lidar_name == "M10_DOUBLE") {
        PACKET_SIZE = 300;
        package_points_ = 70;
        data_bits_start_ = 8;
        degree_bits_start_ = 4;
        rpm_bits_start_ = 6;
        points_size_ = 3000;
        baud_rate_ = 921600;
    } else if (lidar_name == "N10_P") {
        PACKET_SIZE = 108;
        package_points_ = 16;
        data_bits_start_ = 7;
        degree_bits_start_ = 5;
        end_degree_bits_start_ = 105;
        baud_rate_ = 460800;
        points_size_ = 2000;
        use_gps_ts = false;
        compensation = false;
    } else if (lidar_name == "L10") {
        PACKET_SIZE = 58;
        package_points_ = 16;
        data_bits_start_ = 7;
        degree_bits_start_ = 5;
        end_degree_bits_start_ = 55;
        baud_rate_ = 230400;
        points_size_ = 2000;
        use_gps_ts = false;
        compensation = false;
    } else
    */
    // if (lidar_name == "M10_P") {
    packet_size_ = 160;
    package_points_ = 70;
    data_bits_start_ = 8;
    degree_bits_start_ = 4;
    rpm_bits_start_ = 6;
    baud_rate_ = 500000;
    max_points_count_ = 2000;
    // TODO: rework this so that we only publish the actual data and we should shrink to fit
    // scan is 30 degrees per sample so 12 points per scan, reserve double that.
    scan_points_.reserve(package_points_ * 24);
    scan_points_shared_.reserve(package_points_ * 24);
    scan_points_to_pub_.reserve(package_points_ * 24);
    // }
    RCLCPP_INFO_STREAM(this->get_logger(), "Lidar is " << lidar_name_.c_str());

    if (pubScan_) scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);
    if (pubPointCloud2_)
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 10);
    difop_switch_ = this->create_subscription<std_msgs::msg::Int8>(
        "lslidar_order", 1, std::bind(&LslidarDriver::lidar_order, this, std::placeholders::_1)); // 转速输入
    read_serial_timer_
        = this->create_wall_timer(std::chrono::milliseconds(3), std::bind(&LslidarDriver::polling, this));
    return true;
}

void LslidarDriver::lidar_difop() {
    if (lidar_name_ == "L10" || lidar_name_ == "N10" || lidar_name_ == "N10_P") return;
    if (interface_selection_ == "net") msop_input_->UDP_difop();
    else {
        for (int k = 0; k < 10; k++) {
            unsigned char data[188] = { 0x00 };
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[2] = 0x55;
            data[184] = 0x08;
            data[185] = 0x01;
            data[186] = 0xFA;
            data[187] = 0xFB;
            int rtn = serial_->send((const char*)data, 188);
            if (rtn < 0) printf("start scan error !\n");
            else return;
        }
    }
    return;
}

void LslidarDriver::lidar_order(const std_msgs::msg::Int8::SharedPtr msg) {
    if (lidar_name_ == "L10") return;
    int i = msg->data;
    if (i == 0) is_start_ = false;
    else is_start_ = true;
    if (interface_selection_ == "net") msop_input_->UDP_order(*msg);
    else {
        int i = msg->data;
        for (int k = 0; k < 10; k++) {
            int rtn;
            unsigned char data[188] = { 0x00 };
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[2] = 0x55;
            data[186] = 0xFA;
            data[187] = 0xFB;

            if (lidar_name_ == "M10" || lidar_name_ == "M10_GPS" || lidar_name_ == "M10_P"
                || lidar_name_ == "M10_DOUBLE") {
                if (i <= 1) { // lidar start/stop
                    data[184] = 0x01;
                    data[185] = char(i);
                } else if (i == 2) { // point cloud unfiltered
                    data[181] = 0x0A;
                    data[184] = 0x06;
                    if (is_start_) data[185] = 0x01;
                } else if (i == 3) { // point cloud normal filtering
                    data[181] = 0x0B;
                    data[184] = 0x06;
                    if (is_start_) data[185] = 0x01;
                } else if (i == 4) { // close range filtering
                    data[181] = 0x0C;
                    data[184] = 0x06;
                    if (is_start_) data[185] = 0x01;
                } else if (i == 100) { // device package
                    data[184] = 0x08;
                    data[185] = 0x01;
                } else return;
            } else if (lidar_name_ == "M10_PLUS") {
                data[184] = 0x0A;
                data[185] = 0x01;
                if (i == 5) {
                    data[141] = 0x01;
                    data[142] = 0x2c;
                } else if (i == 6) {
                    data[141] = 0x01;
                    data[142] = 0x68;
                } else if (i == 8) {
                    data[141] = 0x01;
                    data[142] = 0xe0;
                } else if (i == 10) {
                    data[141] = 0x02;
                    data[142] = 0x58;
                } else if (i == 12) {
                    data[141] = 0x02;
                    data[142] = 0xd0;
                } else if (i == 15) {
                    data[141] = 0x03;
                    data[142] = 0x84;
                } else if (i == 20) {
                    data[141] = 0x04;
                    data[142] = 0xb0;
                } else if (i <= 1) {
                    data[184] = 0x01;
                    data[185] = char(i);
                } else if (i == 100) // 接收设备包
                {
                    data[184] = 0x08;
                    data[185] = 0x01;
                } else return;
            } else if (lidar_name_ == "N10" || lidar_name_ == "N10_P") {
                if (i <= 1) {
                    data[185] = char(i);
                    data[184] = 0x01;
                } else if (i >= 6 && i <= 12) {
                    data[172] = char(i);
                    data[184] = 0x0a;
                    data[185] = 0X01;
                } else return;
            }
            rtn = serial_->send((const char*)data, 188);
            if (rtn < 0) printf("start scan error !\n");
            else {
                if (i == 1) usleep(1000000); // 1.0s
                if (i == 0) is_start_ = false;
                if (i == 1) is_start_ = true;
                return;
            }
        }
        return;
    }
}

void LslidarDriver::open_serial() {
    diagnostics_.setHardwareID("Lslidar");
    this->get_parameter("serial_port_", serial_port_);
    serial_ = LSIOSR::instance(serial_port_, baud_rate_);
    int errcode = serial_->init();
    if (errcode != 0) {
        RCLCPP_ERROR(this->get_logger(), "open_port %s ERROR - %s: %s", serial_port_.c_str(), strerrorname_np(errcode),
            strerror(errcode));
        rclcpp::shutdown();
        exit(0);
    }
    RCLCPP_INFO(this->get_logger(), "open_port %s OK !\n", serial_port_.c_str());
}

bool LslidarDriver::createRosIO() {
    UDP_PORT_NUMBER = 2368;
    this->declare_parameter<int>("msop_port", 2368);
    this->get_parameter("msop_port", UDP_PORT_NUMBER);
    RCLCPP_INFO_STREAM(this->get_logger(), "Opening UDP socket: port " << UDP_PORT_NUMBER);
    dump_file_ = std::string("");
    this->declare_parameter<std::string>("pcap", "");
    this->get_parameter("pcap", dump_file_);
    // ROS diagnostics
    diagnostics_.setHardwareID("Lslidar");

    const double diag_freq = 12 * 24;
    diag_max_freq_ = diag_freq;
    diag_min_freq_ = diag_freq;
    RCLCPP_INFO(this->get_logger(), "expected frequency: %.3f (Hz)", diag_freq);

    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("lslidar_packets", diagnostics_,
        FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10), TimeStampStatusParam()));

    int hz = 10;
    if (lidar_name_ == "M10_P") hz = 12;
    else if (lidar_name_ == "M10_PLUS") hz = 20;

    double packet_rate = hz * 24;
    if (dump_file_ != "") {
        msop_input_.reset(new lslidar_driver::InputPCAP(this, UDP_PORT_NUMBER, packet_rate, dump_file_));
    } else {
        msop_input_.reset(new lslidar_driver::InputSocket(this, UDP_PORT_NUMBER));
    }

    // Output
    return true;
}

size_t LslidarDriver::GetScanToPublish(rclcpp::Time& scan_time, float& scan_duration) {
    size_t pub_sample_count { 0 };
    {
        boost::unique_lock<boost::mutex> lock(mutex_);
        std::swap(scan_points_to_pub_, scan_points_shared_);
        pub_sample_count = pub_sample_count_shared_;
    }
    scan_time = pre_time_;
    scan_duration = time_.seconds() - pre_time_.seconds();
    return pub_sample_count;
}

uint64_t LslidarDriver::get_gps_stamp(struct tm t) {

    uint64_t ptime = static_cast<uint64_t>(timegm(&t));
    return ptime;
}

bool LslidarDriver::initialize() {
    if (!loadParameters()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot load all required ROS parameters...");
        return false;
    }
    if (interface_selection_ == "net") {
        if (!createRosIO()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot create all ROS IO...");
            return false;
        }
    } else {
        in_file_name_ = std::string("");
        this->declare_parameter<std::string>("in_file_name", "");
        this->get_parameter("in_file_name", in_file_name_);
        if (in_file_name_ == "") open_serial();
        else {
            RCLCPP_INFO_STREAM(this->get_logger(), "Opening txt file " << in_file_name_.c_str());
            std::ifstream file_reader(in_file_name_);
            if (!file_reader.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Cannot open the file");
                return false;
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "Initialised lslidar without error");
    return true;
}

void LslidarDriver::recvThread_crc(int& count, int& link_time) {
    if (count <= 0) link_time++;
    else link_time = 0;

    if (link_time > 150) {
        int ret = serial_->close();
        if (ret < 0) {
            RCLCPP_ERROR(
                this->get_logger(), "Failed to close serial port - %s: %s", strerrorname_np(errno), strerror(errno));
        }
        ret = serial_->init();
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "serial open fail");
            usleep(200000);
        }
        link_time = 0;
    }
}

int LslidarDriver::SerialReadBytes(uint8_t buf[], size_t n, int timeout) {
    size_t total_count { 0 };
    int link_time { 0 };
    auto timeout_ms = std::chrono::milliseconds(timeout);
    auto end = std::chrono::steady_clock::now() + timeout_ms;
    do {
        int curr_count = serial_->read(&buf[total_count], n - total_count);
        if (curr_count >= 0) {
            total_count += curr_count;
        } else {
            // error condition
            RCLCPP_ERROR(this->get_logger(), "Serial read error - %s: %s", strerrorname_np(errno), strerror(errno));
            return -1;
        }

        LslidarDriver::recvThread_crc(curr_count, link_time);
    } while (total_count < n && std::chrono::steady_clock::now() < end);
    return total_count;
}

bool LslidarDriver::SeekToMagicBytes(uint8_t buf[]) {
    bool found { false };
    while (!found) {
        while (buf[0] != 0xA5) {
            // RCLCPP_INFO(this->get_logger(), "buf[0] = %c", buf[0]);
            if (SerialReadBytes(buf, 1) <= 0) return false;
        }
        if (SerialReadBytes(&buf[1], 1) <= 0) return false;
        if (buf[1] == 0x5A) {
            found = true;
        } else {
            // lone stray 0xA5 probably from data, not a start of packet
            // reread the bytes.
            buf[0] = 0;
            RCLCPP_ERROR(this->get_logger(), "unexpected second magic byte, possible desync");
        }
    }

    return found;
}

int LslidarDriver::GetCurrentRxQueueSize() {
    int rx_queue_count = serial_->GetRxQueueCurrentSize();
    if (rx_queue_count < 0) {
        RCLCPP_ERROR(
            this->get_logger(), "Failed to read kernel queue size - %s: %s", strerrorname_np(errno), strerror(errno));
        return -1;
    }
    // RCLCPP_DEBUG(this->get_logger(), "bytes in kernel queue: %d", rx_queue_count);
    return rx_queue_count;
}

void LslidarDriver::ClearInternalState() {
    if (serial_->flushinput() < 0) {
        RCLCPP_ERROR(
            this->get_logger(), "Failed to flush kernel queue - %s: %s", strerrorname_np(errno), strerror(errno));
    }
    scan_points_.assign(max_points_count_, { 0, 0, 0 });
    // scan_points_shared_.assign(max_points_count_, { 0, 0, 0 });
    // scan_points_to_pub_.assign(max_points_count_, { 0, 0, 0 });
    idx_ = 0;
    degree_compensation_ = 0;
    angle_covered_by_scan_points_ = 0.f;
}

int LslidarDriver::receive_data(std::vector<uint8_t>& dst) {
    // Get the rx queue size BEFORE reading from it
    int queue_size { GetCurrentRxQueueSize() };
    if (queue_size < 0) {
        return 0;
    }
    if (queue_size < max_packet_len_) {
        // not guaranteed to get a full packet. Try again later.
        RCLCPP_DEBUG(this->get_logger(), "Not enough bytes to form a packet (%d), retrying next iteration", queue_size);
        curr_failed_reads_++;
        if (curr_failed_reads_ > max_consecutive_failed_reads_) {
            // some connection error. Close and reopen the connection.
            RCLCPP_WARN(this->get_logger(), "Possible connection failure. Resetting the serial connection.");
            int ret = serial_->close();
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to close serial port - %s: %s", strerrorname_np(errno),
                    strerror(errno));
            }
            serial_->init();
            ClearInternalState();
            curr_failed_reads_ = 0;
        }
        return 0;
    }

    // flush queue
    // flush all the buffers
    // idx_ = 0
    // degree_compensation_ = 0
    // queue length is technically 4096 but kernel will drop all bytes that are not a line break
    // give 1 packet length of buffer here
    else if (queue_size >= (3907)) {
        RCLCPP_WARN(this->get_logger(), "Kernel buffer full. Clearing internal state as a precaution.");
        ClearInternalState();
        curr_failed_reads_ = 0;
        return 0;
    }
    curr_failed_reads_ = 0;

    int len = 0;
    uint8_t magic_buf[2] { 0, 0 };
    uint8_t size_buf[2] { 0, 0 };

    // read the first two start of frame bytes
    if (!SeekToMagicBytes(magic_buf)) {
        return 0;
    }

    // next two bytes determine size of frame
    if (SerialReadBytes(size_buf, 2) < 2) {
        return 0;
    }

    // if (lidar_name == "M10") len = 92;
    // else if (lidar_name == "M10_GPS") len = 102;
    // else if (lidar_name == "N10_P") len = 108;
    // else if (lidar_name == "N10" || lidar_name == "L10") len = packet_bytes[2];
    // else {
    len = (size_buf[0] << 8) + size_buf[1];
    // typical length is 156 to 188 bytes long (compensation? or something)
    if (len > max_packet_len_ || len < min_packet_len_) {
        RCLCPP_WARN(this->get_logger(), "Bad value for packet length. len =  %d. Skipping sample", len);
        return 0;
    }
    // }
    if (lidar_name_ == "M10" || lidar_name_ == "M10_DOUBLE" || lidar_name_ == "M10_GPS" || lidar_name_ == "M10_P"
        || lidar_name_ == "M10_PLUS") {
        if (size_buf[0] == 0x55 && size_buf[1] == 0x00) len = 188;
    }
    RCLCPP_DEBUG(this->get_logger(), "len = %d", len);

    // memset 0 the vector
    dst.assign(len, 0);
    dst[0] = magic_buf[0];
    dst[1] = magic_buf[1];
    dst[2] = size_buf[0];
    dst[3] = size_buf[1];
    if (SerialReadBytes(&(dst.data()[4]), len - 4) < len - 4) {
        // skip if we have some issues with reading
        RCLCPP_WARN(this->get_logger(), "Error while trying to read data size %d. Skipping packet.", len);
        return 0;
    }

    // if we want to write data to a file
    // out_stream_.write(reinterpret_cast<const char*>(dst.data()), len);

    // if (lidar_name == "N10" || lidar_name == "L10" || lidar_name == "N10_P") {
    //     if (packet_bytes[PACKET_SIZE - 1] != N10_CalCRC8(packet_bytes, PACKET_SIZE - 1)) return 0;
    // }
    return len;
}

uint8_t LslidarDriver::N10_CalCRC8(uint8_t* p, int len) {
    uint8_t crc = 0;
    int sum = 0;

    for (int i = 0; i < len; i++) {
        sum += uint8_t(p[i]);
    }
    crc = sum & 0xff;
    return crc;
}

void LslidarDriver::difop_processing(const std::vector<uint8_t>& packet_bytes) // 处理设备包的数据
{
    int s = packet_bytes[173];
    int z = packet_bytes[174];
    int degree_temp = s & 0x7F;
    int sign_temp = s & 0x80;
    degree_compensation_ = double(degree_temp * 256 + z) / 100.f;
    if (sign_temp) degree_compensation_ = -degree_compensation_;
    first_compensation_ = false;
    printf("degree_compensation = %f\n", degree_compensation_);
    return;
}

void LslidarDriver::ProcessPacket(const std::vector<uint8_t>& buf) // 处理每一包的数据
{
    // packet is in some form
    // - start heading
    // - samples
    // - we should be able to get some sort of reading what is the deg interval
    //
    if (buf.size() == 0) {
        return;
    }
    double start_angle;
    // double end_degree;
    boost::posix_time::ptime t1, t2;
    t1 = boost::posix_time::microsec_clock::universal_time();

    int s = buf[degree_bits_start_];
    int z = buf[degree_bits_start_ + 1];
    start_angle = (s * 256 + z) / 100.f + degree_compensation_;
    start_angle = (start_angle < 0) ? start_angle + 360 : start_angle;
    start_angle = (start_angle > 360) ? start_angle - 360 : start_angle;

    constexpr double packet_angle_interval { 15.f };

    // not relevant
    /*
    if (lidar_name == "N10" || lidar_name == "L10") {
        int s_e = buf[end_degree_bits_start];
        int z_e = buf[end_degree_bits_start + 1];

        end_degree = (s_e * 256 + z_e) / 100.f;
        end_degree = (end_degree > 360) ? end_degree - 360 : end_degree;

        if (degree > end_degree) degree_interval = end_degree + 360 - degree;
        else degree_interval = end_degree - degree;
    }
    */

    if (lidar_name_ == "M10_PLUS" || lidar_name_ == "M10_P") {
        packet_size_ = buf.size();
        package_points_ = (packet_size_ - 20) / 2;
    }
    int invalidValue = 0;
    constexpr int point_len = 2;
    // if (lidar_name == "N10" || lidar_name == "L10") point_len = 3;

    /*
    if (lidar_name == "M10_GPS" || lidar_name == "M10") {
        int err_data_84 = buf[84];
        int err_data_85 = buf[85];
        if ((err_data_84 * 256 + err_data_85) == 0xFFFF || buf[86] >= 0xF5) {
            buf[86] = 0xFF;
            buf[87] = 0xFF;
        }
    }
    */

    for (int num = 0; num < point_len * package_points_; num += point_len) {
        int s = buf[num + data_bits_start_];
        int z = buf[num + data_bits_start_ + 1];
        if ((s * 256 + z) == 0xFFFF) invalidValue++;
    }

    if (use_gps_ts_ && lidar_name_ != "N10") {
        pTime_.tm_year = buf[packet_size_ - 12] + 2000 - 1900; // x+2000
        pTime_.tm_mon = buf[packet_size_ - 11] - 1; // 1-12
        pTime_.tm_mday = buf[packet_size_ - 10]; // 1-31
        pTime_.tm_hour = buf[packet_size_ - 9]; // 0-23
        pTime_.tm_min = buf[packet_size_ - 8]; // 0-59
        pTime_.tm_sec = buf[packet_size_ - 7]; // 0-59
        sub_second_ = (buf[packet_size_ - 6] * 256 + buf[packet_size_ - 5]) * 1000000
            + (buf[packet_size_ - 4] * 256 + buf[packet_size_ - 3]) * 1000;
        sweep_end_time_gps_ = get_gps_stamp(pTime_);
        sweep_end_time_hardware_ = sub_second_ % 1000000000;
    }

    // this becomes the number of valid points
    int valid_points_count = package_points_ - invalidValue;
    // if (lidar_name == "N10" || lidar_name == "L10") invalidValue--;
    if (valid_points_count <= 1) {
        RCLCPP_WARN(this->get_logger(), "Number of valid samples <= 1");
        return;
    }
    // scan_points_.resize(package_points_);
    const double sample_angle_increment { packet_angle_interval / package_points_ };

    // package_points_ is number of samples
    for (int i = 0; i < package_points_; i++) {
        // 1 datapoint is 2 bytes
        uint8_t msb = buf[i * point_len + data_bits_start_];
        uint8_t lsb = buf[i * point_len + data_bits_start_ + 1];
        // if (lidar_name == "N10" || lidar_name == "L10") y = buf[num * point_len + data_bits_start + 2];
        int dist_temp = msb & 0x7F;
        int inten_temp = msb & 0x80;

        const double angle_raw { start_angle + (sample_angle_increment * i) };
        const double sample_angle { angle_raw > 360.f ? angle_raw - 360.f : angle_raw };

        // publish existing scan points when we have one revolution's worth of packets
        // if ((scan_points_[idx_].degree < last_degree_ && scan_points_[idx_].degree < 15 && last_degree_ > 345)
        //     || idx_ >= max_points_count_ - 1) {
        if (angle_covered_by_scan_points_ + sample_angle_increment >= (angle_able_max_ - angle_able_min_)) {

            // RCLCPP_INFO(this->get_logger(), "publishing. idx_ = %d", idx_);

            // save this in a local variable bc race conditions
            const size_t pub_sample_count = idx_;
            scan_points_.resize(pub_sample_count);
            for (size_t k = 0; k < pub_sample_count; k++) {
                if (scan_points_[k].range < min_range_ || scan_points_[k].range > max_range_) {
                    // RCLCPP_WARN(this->get_logger(), "scan_points_[%ld].range = %f", k, scan_points_[k].range);
                    scan_points_[k].range = 0;
                }
            }

            {
                boost::unique_lock<boost::mutex> lock(mutex_);
                scan_points_shared_.resize(pub_sample_count);
                // just swap the size, cap and data ptr here
                std::swap(scan_points_, scan_points_shared_);
                pub_sample_count_shared_ = pub_sample_count;
                data_ready_ = true;
            }
            // guaranteed that pub_sample_count_shared_ is reassigned.
            // publish the message
            pubscan_cond_.notify_one();
            idx_ = 0;
            scan_points_.assign(max_points_count_, { 0.f, 0.f, 0.f });
            pre_time_ = time_;
            time_ = get_clock()->now();
            angle_covered_by_scan_points_ = 0.f;
        }

        // add the current sample to the scan points
        if ((msb * 256 + lsb) != 0xFFFF) {
            /*
            if (lidar_name == "N10" || lidar_name == "L10") {
                scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
                scan_points_[idx].intensity = int(y);
            } else
            */

            // if ((lidar_name_ == "M10_P" || lidar_name_ == "M10_PLUS") && !high_reflection_) {
            if (!high_reflection_) {
                scan_points_[idx_].range = double(msb * 256 + (lsb)) / 1000.f;
                scan_points_[idx_].intensity = 0;
            } else {
                scan_points_[idx_].range = double(dist_temp * 256 + (lsb)) / 1000.f;
                if (inten_temp) scan_points_[idx_].intensity = 255;
                else scan_points_[idx_].intensity = 0;
            }

            // if (sample_angle > 360.0) scan_points_[idx_].degree = sample_angle - 360;
            // else scan_points_[idx_].degree = sample_angle;
            scan_points_[idx_].degree = sample_angle;
        } else {
            RCLCPP_WARN(this->get_logger(), "sample idx %d invalid", idx_);
        }

        angle_covered_by_scan_points_ += sample_angle_increment;
        idx_++;
    }
    // this makes absolutely 0 sense whatsoever
    // packet_bytes = { 0x00 };
    // if (packet_bytes) {
    //     packet_bytes = NULL;
    //     delete packet_bytes;
    // }
}

/*
void LslidarDriver::data_processing_2(unsigned char* packet_bytes,
    int len) // 处理每一包的数据
{
    double degree;
    double end_degree;
    double degree_interval = 15.0;
    boost::posix_time::ptime t1, t2;
    t1 = boost::posix_time::microsec_clock::universal_time();

    int s = packet_bytes[degree_bits_start];
    int z = packet_bytes[degree_bits_start + 1];

    degree = (s * 256 + z) / 100.f + degree_compensation;
    degree = (degree < 0) ? degree + 360 : degree;
    degree = (degree > 360) ? degree - 360 : degree;
    // not relevant
    if (lidar_name == "N10_P") {
        int s_e = packet_bytes[end_degree_bits_start];
        int z_e = packet_bytes[end_degree_bits_start + 1];

        end_degree = (s_e * 256 + z_e) / 100.f;
        end_degree = (end_degree > 360) ? end_degree - 360 : end_degree;

        if (degree > end_degree) degree_interval = end_degree + 360 - degree;
        else degree_interval = end_degree - degree;
    }

    // boost::unique_lock<boost::mutex> lock(mutex_);
    if (lidar_name == "M10_DOUBLE") {
        PACKET_SIZE = len;
        package_points = (PACKET_SIZE - 20) / 4;
    }
    int invalidValue = 0;
    int point_len = 4;
    // if (lidar_name == "N10_P") point_len = 6;

    for (int num = 0; num < point_len * package_points; num += point_len) {
        int s = packet_bytes[num + data_bits_start];
        int z = packet_bytes[num + data_bits_start + 1];
        if ((s * 256 + z) == 0xFFFF) invalidValue++;
    }

    if (use_gps_ts) {
        pTime.tm_year = packet_bytes[PACKET_SIZE - 12] + 2000 - 1900; // x+2000
        pTime.tm_mon = packet_bytes[PACKET_SIZE - 11] - 1; // 1-12
        pTime.tm_mday = packet_bytes[PACKET_SIZE - 10]; // 1-31
        pTime.tm_hour = packet_bytes[PACKET_SIZE - 9]; // 0-23
        pTime.tm_min = packet_bytes[PACKET_SIZE - 8]; // 0-59
        pTime.tm_sec = packet_bytes[PACKET_SIZE - 7]; // 0-59
        sub_second = (packet_bytes[PACKET_SIZE - 6] * 256 + packet_bytes[PACKET_SIZE - 5]) * 1000000
            + (packet_bytes[PACKET_SIZE - 4] * 256 + packet_bytes[PACKET_SIZE - 3]) * 1000;
        sweep_end_time_gps = get_gps_stamp(pTime);
        sweep_end_time_hardware = sub_second % 1000000000;
    }
    invalidValue = package_points - invalidValue;
    if (lidar_name == "N10_P") invalidValue--;
    if (invalidValue <= 1) {
        delete packet_bytes;
        return;
    }

    for (int num = 0; num < package_points; num++) {
        int s = packet_bytes[num * point_len + data_bits_start];
        int z = packet_bytes[num * point_len + data_bits_start + 1];
        int y = 0;
        if (lidar_name == "N10_P") y = packet_bytes[num * point_len + data_bits_start + 2];

        if ((s * 256 + z) != 0xFFFF) {
            scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
            if (lidar_name == "N10_P") scan_points_[idx].intensity = int(y);
            else scan_points_[idx].intensity = 0;
            s = packet_bytes[num * point_len + data_bits_start + point_len / 2];
            z = packet_bytes[num * point_len + data_bits_start + point_len / 2 + 1];
            if (lidar_name == "N10_P") y = packet_bytes[num * point_len + data_bits_start + point_len / 2 + 2];

            scan_points_[idx + 3000].range = double(s * 256 + (z)) / 1000.f;
            if (lidar_name == "N10_P") scan_points_[idx + 3000].intensity = int(y);
            else scan_points_[idx + 3000].intensity = 0;

            if ((degree + (degree_interval / invalidValue * num)) > 360)
                scan_points_[idx].degree = degree + (degree_interval / invalidValue * num) - 360;
            else scan_points_[idx].degree = degree + (degree_interval / invalidValue * num);
        } else continue;
        if (((scan_points_[idx].degree < last_degree && scan_points_[idx].degree < 5 && last_degree > 355)
                || idx >= points_size_)
            && idx > 10) {
            last_degree = scan_points_[idx].degree;
            count_num = idx;
            idx = 0;
            for (int k = 0; k < count_num; k++) {
                if (angle_able_max > 360) {
                    if ((360 - scan_points_[k].degree) > (angle_able_max - 360)
                        && (360 - scan_points_[k].degree) < angle_able_min) {
                        scan_points_[k].range = 0;
                        scan_points_[k + 3000].range = 0;
                    }
                } else {
                    if ((360 - scan_points_[k].degree) > angle_able_max
                        || (360 - scan_points_[k].degree) < angle_able_min) {
                        scan_points_[k].range = 0;
                        scan_points_[k + 3000].range = 0;
                    }
                }
                if (scan_points_[k].range < min_range || scan_points_[k].range > max_range) scan_points_[k].range = 0;
                if (scan_points_[k + 3000].range < min_range || scan_points_[k + 3000].range > max_range)
                    scan_points_[k + 3000].range = 0;
            }
            boost::unique_lock<boost::mutex> lock(mutex_);
            scan_points_bak_.resize(scan_points_.size());
            scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
            for (long unsigned int k = 0; k < scan_points_.size(); k++) {
                scan_points_[k].range = 0;
                scan_points_[k].degree = 0;
                scan_points_[k].intensity = 0;
            }
            pre_time_ = time_;
            lock.unlock();
            pubscan_cond_.notify_one();
            time_ = get_clock()->now();
        } else {
            last_degree = scan_points_[idx].degree;
            idx++;
        }
    }
    packet_bytes = { 0x00 };
    if (packet_bytes) {
        packet_bytes = NULL;
        delete packet_bytes;
    }
}
*/

void LslidarDriver::pubScanThread() {

    // should_shutdown_ checked in two places
    // while loop: if the flag is set while publishing
    // pubscan_cond_.wait(): if the flag is set while waiting for new data

    while (!should_shutdown_) {
        {
            boost::unique_lock<boost::mutex> lock(mutex_);
            while (!data_ready_ && !should_shutdown_) {
                pubscan_cond_.wait(lock);
                if (should_shutdown_) {
                    return;
                }
            }
            data_ready_ = false;
        }
        // these are not our models
        /*
        if (lidar_name == "N10_P" || lidar_name == "M10_DOUBLE") {
            if (pubScan) {
                auto scan = sensor_msgs::msg::LaserScan::UniquePtr(new sensor_msgs::msg::LaserScan());
                ////int scan_num = count_num * 2;
                int scan_num = count_num;

                std::vector<ScanPoint> points;
                rclcpp::Time start_time;
                float scan_time;
                this->getScan(points, start_time, scan_time);
                scan->header.frame_id = frame_id;
                if (use_gps_ts) {
                    scan->header.stamp = rclcpp::Time(sweep_end_time_gps, sweep_end_time_hardware);
                } else {
                    scan->header.stamp = this->now(); // timestamp will obtained from sweep data stamp
                }

                scan->angle_min = 0;
                scan->angle_max = 2 * M_PI;
                scan->angle_increment = 2 * M_PI / (double)(count_num);
                scan->range_min = min_range;
                scan->range_max = max_range;
                scan->ranges.reserve(scan_num);
                scan->ranges.assign(scan_num, std::numeric_limits<float>::infinity());
                scan->intensities.reserve(scan_num);
                scan->intensities.assign(scan_num, std::numeric_limits<float>::infinity());
                // scan->scan_time = scan_time;
                // scan->time_increment = scan_time / (double)(count_num);

                for (int k = 0; k < scan_num; k++) {
                    scan->ranges[k] = std::numeric_limits<float>::infinity();
                    scan->intensities[k] = 0;
                }

                for (int i = 0; i < count_num; i++) {
                    int point_idx = round((360 - points[i].degree) * count_num / 360);
                    if (points[i].range == 0.0) {
                        scan->ranges[point_idx] = std::numeric_limits<float>::infinity();
                        scan->intensities[point_idx] = 0;
                    } else {
                        double dist = points[i].range;
                        scan->ranges[point_idx] = (float)dist;
                        scan->intensities[point_idx] = points[i].intensity;
                    }

                    if (truncated_mode_) {
                        int len = sizeof(scan_crop_max) / sizeof(scan_crop_max[0]);
                        for (int j = 0; j < len; ++j) {
                            if ((point_idx >= (scan_crop_min[j] * count_num / 360))
                                && (point_idx <= (scan_crop_max[j] * count_num / 360))) {
                                scan->ranges[point_idx] = std::numeric_limits<float>::infinity();
                                scan->intensities[point_idx] = 0;
                            }
                        }
                    }
                    //
                    // if (points[i + 3000].range == 0.0)
                    // {
                    //         scan->ranges[point_idx + count_num] = std::numeric_limits<float>::infinity();
                    //         scan->intensities[point_idx + count_num] = 0;
                    // }
                    // else
                    // {
                    //         double dist = points[i+3000].range;
                    //         scan->ranges[point_idx + count_num] = (float)dist;
                    //         scan->intensities[point_idx + count_num] = points[i + 3000].intensity;
                    // }
                }
                scan_pub->publish(std::move(scan));
            }
            if (pubPointCloud2) {
                std::vector<ScanPoint> points;
                rclcpp::Time start_time;
                float scan_time;
                this->getScan(points, start_time, scan_time);
                VPointCloud::Ptr point_cloud(new VPointCloud());
                if (use_gps_ts) {
                    start_time = rclcpp::Time(sweep_end_time_gps, sweep_end_time_hardware);
                }
                double timestamp = start_time.seconds();
                point_cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
                point_cloud->header.frame_id = frame_id;
                point_cloud->height = 1;
                // printf("now = %f\n",timestamp);
                for (uint16_t i = 0; i < count_num; i++) {
                    // printf("degree = %f\n",points[i].degree);
                    double degree = 360.0 - points[i].degree;
                    bool pass_point = false;
                    if (angle_able_max < 360) {
                        if (degree < angle_able_min || degree > angle_able_max) pass_point = true;
                    } else {
                        if (degree < angle_able_min && degree > (angle_able_max - 360)) pass_point = true;
                    }
                    if (points[i].range < 0.001) pass_point = true;
                    if (!pass_point) {
                        // printf("degree = %f\n",degree);
                        // printf("angle_able_min = %f\nangle_able_max=%f\n",angle_able_min,angle_able_max);
                        VPoint point;
                        int point_idx = round(degree * count_num / 360);
                        point.timestamp = timestamp - point_idx * (scan_time / count_num);
                        // printf("timestamp = %f\n",point.timestamp);
                        point.x = points[i].range * cos(M_PI / 180 * points[i].degree);
                        point.y = -points[i].range * sin(M_PI / 180 * points[i].degree);
                        point.z = 0;
                        point.intensity = points[i].intensity;
                        point_cloud->points.push_back(point);
                        ++point_cloud->width;
                    }
                    if (points[i + 3000].range < 0.001) pass_point = true;
                    if (!pass_point) {
                        // printf("degree = %f\n",degree);
                        // printf("angle_able_min = %f\nangle_able_max=%f\n",angle_able_min,angle_able_max);
                        VPoint point;
                        int point_idx = round(degree * count_num / 360);
                        point.timestamp = timestamp - point_idx * (scan_time / count_num);
                        // printf("timestamp = %f\n",point.timestamp);
                        point.x = points[i + 3000].range * cos(M_PI / 180 * points[i].degree);
                        point.y = -points[i + 3000].range * sin(M_PI / 180 * points[i].degree);
                        point.z = 0;
                        point.intensity = points[i + 3000].intensity;
                        point_cloud->points.push_back(point);
                        ++point_cloud->width;
                    }
                }
                sensor_msgs::msg::PointCloud2 pc_msg;
                pcl::toROSMsg(*point_cloud, pc_msg);
                point_cloud_pub->publish(pc_msg);
            }
        } else {
            */
        constexpr double twopi = 2 * M_PI;
        constexpr double deg2rad_scale = twopi / 360;
        size_t pub_sample_count { 0 };
        if (pubScan_) {
            auto scan = sensor_msgs::msg::LaserScan::UniquePtr(new sensor_msgs::msg::LaserScan());
            rclcpp::Time start_time;
            float scan_time;
            pub_sample_count = GetScanToPublish(start_time, scan_time);
            if (scan_points_to_pub_.size() == 0) {
                RCLCPP_WARN(this->get_logger(), "No scan to get. Not publishing.");
                continue;
            }

            scan->header.frame_id = frame_id_;
            if (use_gps_ts_) {
                scan->header.stamp = rclcpp::Time(sweep_end_time_gps_, sweep_end_time_hardware_);
            } else {
                scan->header.stamp = this->now(); // timestamp will obtained from sweep data stamp
            }

            scan->angle_min = deg2rad_scale * angle_able_min_;
            scan->angle_max = deg2rad_scale * angle_able_max_;
            scan->angle_increment = twopi / (pub_sample_count - 1);
            scan->range_min = min_range_;
            scan->range_max = max_range_;
            scan->ranges.assign(pub_sample_count, std::numeric_limits<float>::infinity());
            scan->intensities.assign(pub_sample_count, 0.f);
            scan->scan_time = scan_time;
            scan->time_increment = scan_time / (pub_sample_count - 1);

            // Get index of min - the vector is in increasing order
            size_t min_index { 0 };
            const auto min_it = std::min_element(scan_points_to_pub_.begin(), scan_points_to_pub_.end(),
                [](const ScanPoint& a, const ScanPoint& b) { return a.degree < b.degree; });

            min_index = std::distance(scan_points_to_pub_.begin(), min_it);

            size_t scan_points_idx = min_index;
            for (size_t i = 0; i < pub_sample_count; ++i) {
                if (scan_points_to_pub_[scan_points_idx].range != 0.f) {
                    scan->ranges[pub_sample_count - 1 - i] = scan_points_to_pub_[scan_points_idx].range;
                    scan->intensities[pub_sample_count - 1 - i]
                        = static_cast<float>(scan_points_to_pub_[scan_points_idx].intensity);
                }
                scan_points_idx++;
                scan_points_idx = scan_points_idx % pub_sample_count;

                // ignore truncated mode
            }

            // int scan_num = ceil((angle_able_max_ - angle_able_min_) / 360 * pub_sample_count) + 1;
            //
            // rclcpp::Time start_time;
            // float scan_time;
            // GetScanToPublish(start_time, scan_time);
            // if (scan_points_to_pub_.size() == 0) {
            //     RCLCPP_WARN(this->get_logger(), "No scan to get.");
            //     return;
            // }
            // scan->header.frame_id = frame_id_;
            // if (use_gps_ts_) {
            //     scan->header.stamp = rclcpp::Time(sweep_end_time_gps_, sweep_end_time_hardware_);
            // } else {
            //     scan->header.stamp = this->now(); // timestamp will obtained from sweep data stamp
            // }
            //
            // if (angle_able_max_ > 360) {
            //     scan->angle_min = 2 * M_PI * (angle_able_min_ - 360) / 360;
            //     scan->angle_max = 2 * M_PI * (angle_able_max_ - 360) / 360;
            // } else {
            //     scan->angle_min = 2 * M_PI * angle_able_min_ / 360;
            //     scan->angle_max = 2 * M_PI * angle_able_max_ / 360;
            // }
            // scan->angle_increment = 2 * M_PI / (double)(pub_sample_count - 1);
            //
            // scan->range_min = min_range_;
            // scan->range_max = max_range_;
            // scan->ranges.reserve(scan_num);
            // scan->ranges.assign(scan_num, std::numeric_limits<float>::infinity());
            // scan->intensities.reserve(scan_num);
            // scan->intensities.assign(scan_num, std::numeric_limits<float>::infinity());
            // scan->scan_time = scan_time;
            // scan->time_increment = scan_time / (double)(pub_sample_count - 1);
            //
            // int start_num = floor(angle_able_min_ * pub_sample_count / 360);
            // int end_num = floor(angle_able_max_ * pub_sample_count / 360);
            //
            // for (size_t i = 0; i < pub_sample_count; i++) {
            //     int point_idx = round((360 - scan_points_to_pub_[i].degree) * pub_sample_count / 360);
            //     if (point_idx < static_cast<int>(end_num - pub_sample_count)) point_idx += pub_sample_count;
            //     point_idx = point_idx - start_num;
            //     if (point_idx < 0 || point_idx > scan_num) continue;
            //     if (scan_points_to_pub_[i].range == 0.0) {
            //         scan->ranges[point_idx] = std::numeric_limits<float>::infinity();
            //     } else {
            //         double dist = scan_points_to_pub_[i].range;
            //         scan->ranges[point_idx] = (float)dist;
            //     }
            //     scan->intensities[point_idx] = scan_points_to_pub_[i].intensity;
            //
            //     if (truncated_mode_) {
            //         int len = sizeof(scan_crop_max) / sizeof(scan_crop_max[0]);
            //         for (int j = 0; j < len; ++j) {
            //             if ((point_idx >= static_cast<int>(scan_crop_min[j] * pub_sample_count / 360))
            //                 && (point_idx <= static_cast<int>(scan_crop_max[j] * pub_sample_count / 360))) {
            //                 scan->ranges[point_idx] = std::numeric_limits<float>::infinity();
            //                 scan->intensities[point_idx] = 0;
            //             }
            //         }
            //     }
            //     // RCLCPP_INFO(this->get_logger(), "%d: range: %f, intensity: %f", point_idx,
            //     // scan->ranges[point_idx],
            //     //     scan->intensities[point_idx]);
            // }

            scan_pub_->publish(std::move(scan));
        }
        if (pubPointCloud2_) {
            std::vector<ScanPoint> points;
            rclcpp::Time start_time;
            float scan_time;
            GetScanToPublish(start_time, scan_time);
            VPointCloud::Ptr point_cloud(new VPointCloud());
            if (use_gps_ts_) {
                start_time = rclcpp::Time(sweep_end_time_gps_, sweep_end_time_hardware_);
            }
            double timestamp = start_time.seconds();
            point_cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
            point_cloud->header.frame_id = frame_id_;
            point_cloud->height = 1;
            for (uint16_t i = 0; i < pub_sample_count; i++) {
                double degree = 360.0 - points[i].degree;
                bool pass_point = false;
                if (angle_able_max_ < 360) {
                    if (degree < angle_able_min_ || degree > angle_able_max_) pass_point = true;
                } else {
                    if (degree < angle_able_min_ && degree > (angle_able_max_ - 360)) pass_point = true;
                }
                if (points[i].range < 0.001) pass_point = true;
                if (!pass_point) {
                    // printf("degree = %f\n",degree);
                    // printf("angle_able_min = %f\nangle_able_max=%f\n",angle_able_min,angle_able_max);
                    VPoint point;
                    int point_idx = round(degree * pub_sample_count / 360);
                    point.timestamp = timestamp - point_idx * (scan_time / pub_sample_count);
                    // printf("timestamp = %f\n",point.timestamp);
                    point.x = points[i].range * cos(M_PI / 180 * points[i].degree);
                    point.y = -points[i].range * sin(M_PI / 180 * points[i].degree);
                    point.z = 0;
                    point.intensity = points[i].intensity;
                    point_cloud->points.push_back(point);
                    ++point_cloud->width;
                }
            }
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            point_cloud_pub_->publish(pc_msg);
        }
        // pub_sample_count_shared_ = 0;
        if (compensation_) {
            lidar_difop();
        }
    }
}

void LslidarDriver::polling() {
    if (!is_start_) return;

    // It will be the processor's responsibility to resize the vector to fit the packet
    int len = 0;
    bool difop = false;
    if (interface_selection_ == "net") {
        auto packet = lslidar_msgs::msg::LslidarPacket::UniquePtr(new lslidar_msgs::msg::LslidarPacket());

        std_msgs::msg::Byte msg;
        while (true) {
            difop = false;
            len = 0;
            // keep reading until full packet received
            len = msop_input_->getPacket(packet);
            if (packet->data[0] == 0x5a) {
                if (lidar_name_ == "N10" || lidar_name_ == "L10") len = 58;
                else if (lidar_name_ == "M10") len = 92;
                else if (lidar_name_ == "N10_P") len = 108;
                else if (lidar_name_ == "M10_GPS") len = 102;
                else {
                    int len_H = packet->data[1];
                    int len_L = packet->data[2];
                    len = len_H * 256 + len_L;
                }
                for (int i = len - 1; i > 0; i--)
                    packet->data[i] = packet->data[i - 1];
                packet->data[0] = 0xa5;
            }

            if (lidar_name_ == "N10" || lidar_name_ == "L10") len = 58;
            else if (lidar_name_ == "M10") len = 92;
            else if (lidar_name_ == "N10_P") len = 108;
            else if (lidar_name_ == "M10_GPS") len = 102;
            else {
                int len_H = packet->data[2];
                int len_L = packet->data[3];
                len = len_H * 256 + len_L;
            }
            RCLCPP_DEBUG(this->get_logger(), "len = %d", len);
            if ((lidar_name_ == "M10" || lidar_name_ == "M10_DOUBLE" || lidar_name_ == "M10_GPS"
                    || lidar_name_ == "M10_P" || lidar_name_ == "M10_PLUS")
                && compensation_) {
                if (packet->data[2] == 0x55 && packet->data[3] == 0x00 && packet->data[186] == 0xFA
                    && packet->data[187] == 0xFB) {
                    len = 188;
                    difop = true;
                }
            }

            if (len <= 0 || len >= 1000 || packet->data[0] != 0xa5 || packet->data[1] != 0x5a) continue;
            for (int i = 0; i < len; i++) {
                serial_read_buf_[i] = packet->data[i];
            }
            if ((lidar_name_ == "N10" || lidar_name_ == "L10" || lidar_name_ == "N10_P")
                && serial_read_buf_[len - 1] != N10_CalCRC8(serial_read_buf_.data(), len - 1))
                continue;
            break;
        }
    } else {
        if (in_file_name_ != "") // read from txt file
        {
            int usleep_time = round(1000000 / 10 / 24) - 135;
            while (true) {
                std::ifstream file_reader(in_file_name_);
                while (file_reader.peek() != EOF) {
                    std::string line;
                    std::getline(file_reader, line, '\n');
                    for (long unsigned int i = 0; i < line.size() - 1; i++) {
                        line[i] = line[i] - 48;
                        if (line[i] > 9) line[i] = line[i] - 39;
                    }

                    for (long unsigned int i = 0; i < (line.size() - 1) / 2; i++) {
                        serial_read_buf_[i] = line[i * 2] * 16 + line[i * 2 + 1];
                    }
                    if (lidar_name_ == "N10" || lidar_name_ == "L10") len = 58;
                    else if (lidar_name_ == "M10") len = 92;
                    else if (lidar_name_ == "N10_P") len = 108;
                    else if (lidar_name_ == "M10_GPS") len = 102;
                    else {
                        int len_H = serial_read_buf_[2];
                        int len_L = serial_read_buf_[3];
                        len = len_H * 256 + len_L;
                    }
                    // if (lidar_name == "N10_P" || lidar_name == "M10_DOUBLE")
                    // LslidarDriver::data_processing_2(packet_bytes, len);
                    // else
                    LslidarDriver::ProcessPacket(serial_read_buf_);
                    usleep(usleep_time);
                }
            }
            return;
        } else {
            difop = false;
            len = LslidarDriver::receive_data(serial_read_buf_);
            if (len == 0) return;
            if ((lidar_name_ == "M10" || lidar_name_ == "M10_DOUBLE" || lidar_name_ == "M10_GPS"
                    || lidar_name_ == "M10_P" || lidar_name_ == "M10_PLUS")
                && compensation_) {
                if (serial_read_buf_[2] == 0x55 && serial_read_buf_[3] == 0x00 && serial_read_buf_[186] == 0xFA
                    && serial_read_buf_[187] == 0xFB)
                    difop = true;
            }
        }
    }
    if (difop) LslidarDriver::difop_processing(serial_read_buf_);
    else {
        // if (lidar_name == "N10_P" || lidar_name == "M10_DOUBLE") LslidarDriver::data_processing_2(packet_bytes, len);
        // else
        LslidarDriver::ProcessPacket(serial_read_buf_);
    }
}

void LslidarDriver::ShutdownDriver() {
    // actually shut down gracefully
    RCLCPP_INFO(this->get_logger(), "Shutting down node");
    {
        boost::unique_lock<boost::mutex> lock(mutex_);
        should_shutdown_ = true;
    }
    pubscan_cond_.notify_one();
    read_serial_timer_->cancel();
    pubscan_thread_->join();
}

} // namespace lslidar_driver

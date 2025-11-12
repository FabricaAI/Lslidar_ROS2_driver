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

#ifndef LSLIDAR_DRIVER_H
#define LSLIDAR_DRIVER_H

#include <netinet/in.h>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "lslidar_msgs/msg/lslidar_packet.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <thread>

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "input.h"
#include "lsiosr.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "time.h"
namespace lslidar_driver {

struct PointXYZIT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

typedef struct {
    double degree;
    double range;
    double intensity;
} ScanPoint;

class LslidarDriver : public rclcpp::Node {
public:
    LslidarDriver();
    LslidarDriver(const rclcpp::NodeOptions& options);
    ~LslidarDriver();

    bool initialize();
    void polling();
    void ShutdownDriver();

    typedef std::shared_ptr<LslidarDriver> LslidarDriverPtr;
    typedef std::shared_ptr<const LslidarDriver> LslidarDriverConstPtr;

private:
    uint64_t get_gps_stamp(struct tm t);
    uint8_t N10_CalCRC8(uint8_t* p, int len);
    bool loadParameters();
    bool createRosIO();
    void open_serial();
    void lidar_difop();
    void lidar_order(const std_msgs::msg::Int8::SharedPtr msg);
    void ProcessPacket(const std::vector<uint8_t>& packet);
    // void data_processing_2(unsigned char* packet_bytes, int len);
    void difop_processing(const std::vector<uint8_t>& packet_bytes);
    void pubScanThread();
    void recvThread_crc(int& count, int& link_time);
    int receive_data(std::vector<uint8_t>& dst);
    size_t GetScanToPublish(rclcpp::Time& scan_time, float& scan_duration);
    int SerialReadBytes(uint8_t buf[], size_t n, int timeout = 100);

    /*
     * @brief Seek to 0xA55A in the stream
     * @return if the magic bytes were found
     */
    bool SeekToMagicBytes(uint8_t buf[]);
    int GetCurrentRxQueueSize();
    void ClearInternalState();

    boost::thread* pubscan_thread_;
    boost::shared_ptr<Input> msop_input_;
    boost::mutex mutex_;
    boost::condition_variable pubscan_cond_;
    int UDP_PORT_NUMBER;
    size_t pub_sample_count_shared_;
    int package_points_;
    int data_bits_start_;
    int degree_bits_start_;
    int end_degree_bits_start_;
    int rpm_bits_start_;
    int baud_rate_;
    int max_points_count_;
    int idx_ = 0;
    int link_time_ = 0;
    const int max_packet_len_ = 188;
    const int min_packet_len_ = 156;
    size_t max_consecutive_failed_reads_;
    size_t curr_failed_reads_ = 0;

    bool use_gps_ts_;
    bool is_start_;
    bool high_reflection_;
    bool compensation_;
    bool first_compensation_ = true;
    bool pubScan_;
    bool pubPointCloud2_;
    bool data_ready_;

    double min_range_;
    double max_range_;
    double angle_disable_min_;
    double angle_disable_max_;
    double angle_able_min_;
    double angle_able_max_;
    double degree_compensation_ = 0.0;
    double angle_covered_by_scan_points_ = 0.f;

    uint16_t packet_size_;
    uint64_t sweep_end_time_gps_;
    uint64_t sweep_end_time_hardware_;
    uint64_t sub_second_;

    std::string frame_id_;
    std::string interface_selection_;
    std::string scan_topic_;
    std::string lidar_name_;
    std::string serial_port_;
    std::string dump_file_;
    std::string pointcloud_topic_;
    std::string in_file_name_;

    tm pTime_;
    rclcpp::Time pre_time_;
    rclcpp::Time time_;
    std::vector<ScanPoint> scan_points_;
    std::vector<ScanPoint> scan_points_shared_;
    std::vector<ScanPoint> scan_points_to_pub_;
    std::vector<uint8_t> serial_read_buf_;
    // Diagnostics updater
    diagnostic_updater::Updater diagnostics_;
    std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
    double diag_min_freq_;
    double diag_max_freq_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr difop_switch_;
    rclcpp::TimerBase::SharedPtr read_serial_timer_;
    LSIOSR* serial_;

    std::ofstream out_stream_;
    bool should_shutdown_ = false;
};
typedef PointXYZIT VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

}; // namespace lslidar_driver
POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_driver::PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(double, timestamp, timestamp))
#endif // _LSLIDAR_DRIVER_H_

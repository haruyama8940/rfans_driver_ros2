/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef _RFANS_DRIVER_H_
#define _RFANS_DRIVER_H_
#include <rclcpp/rclcpp.hpp>
#include "rfans_driver/ioapi.hpp"
#include <stdint.h>
#include "rfans_driver_msgs/srv/rfans_command.hpp"
#include "rfans_driver_msgs/msg/rfans_packet.hpp"
#include "rfans_driver_msgs/msg/rfans_scan.hpp"

namespace rfans_driver
{
class Rfans_Driver : public rclcpp::Node // Nodeを継承する
{
public:
    Rfans_Driver(); // コンストラクタでノード名やオプションを指定する
    ~Rfans_Driver();

    int spinOnce();
    int prog_Set(DEB_PROGRM_S &program);
    int datalevel_Set(DEB_PROGRM_S &program);

private:
    double calcReplayPacketRate();
    void configDeviceParams();
    void setupNodeParams(); // ノードハンドルは不要

    bool worRealtime();
    int spinOnceRealtime();
    bool spinOnceSimu();

    struct
    {
        std::string command_path;
        std::string advertise_path;
        std::string device_ip;
        std::string device_name;
        std::string simu_filepath;
        int dataport;
        int scnSpeed;
        int data_level;
        bool dual_echo;
    } config_;
    rfans_driver::IOAPI *m_devapi;
    rclcpp::Publisher<rfans_driver_msgs::msg::RfansPacket>::SharedPtr m_output; // ROS2のメッセージ型とSharedPtrを使う
    rclcpp::Service<rfans_driver_msgs::srv::RfansCommand>::SharedPtr server_; // ROS2のサービス型とSharedPtrを使う
    rfans_driver_msgs::msg::RfansPacket tmpPacket; // ROS2のメッセージ型を使う
    rfans_driver::InputPCAP *input_;
};

}

#endif //_RFANS_DRIVER_H_

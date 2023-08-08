#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include "rclcpp/rclcpp.hpp"
#include "rfans_driver/rfans_driver.hpp"
#include "rfans_driver_msgs/srv/rfans_command.hpp"
#include "rfans_driver_msgs/msg/rfans_packet.hpp"
#include "rfans_driver_msgs/msg/rfans_scan.hpp"

namespace rfans_driver {

static const size_t packet_size = sizeof(rfans_driver_msgs::msg::RfansPacket().data);
static const int RFANS_PACKET_NUM = 1024;
size_t packet_size_pcap = 1206;

static Rfans_Driver *s_this = NULL;

/** @brief Rfans Command Handle */
// bool CommandHandle(const std::shared_ptr<rfans_driver_msgs::srv::RfansCommand::Request> req,
//                    std::shared_ptr<rfans_driver_msgs::srv::RfansCommand::Response> res)
// {
//     res->status = 1;

//     RCLCPP_INFO(rclcpp::get_logger("rfans_driver"), "request: cmd=%d, speed=%d Hz", req->cmd, req->speed);
//     RCLCPP_INFO(rclcpp::get_logger("rfans_driver"), "sending back response: [%d]", res->status);

//     DEB_PROGRM_S tmpProg;
//     tmpProg.cmdstat = static_cast<DEB_CMD_E>(req->cmd);
//     tmpProg.dataFormat = eFormatCalcData;
//     tmpProg.scnSpeed = req->speed;

//     if (s_this) {
//         s_this->prog_Set(tmpProg);
//     }

//     return true;
// }

class RfansDriverNode : public rclcpp::Node
{
public:
    RfansDriverNode()
        : Node("rfans_driver_node")
    {
        setupNodeParams();
        // server_ = this->create_service<rfans_driver_msgs::srv::RfansCommand>("rfans_driver/" + config_.command_path, &CommandHandle);

        // driver init
        // m_output = this->create_publisher<rfans_driver_msgs::msg::RfansPacket>("rfans_driver/" + config_.advertise_path, RFANS_PACKET_NUM);
        m_output = this->create_publisher<rfans_driver_msgs::msg::RfansPacket>("rfans_driver/" + config_.advertise_path, 10);
        double packet_rate = calcReplayPacketRate();
        rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("rfans_driver");
        if (!config_.simu_filepath.empty()) {
            
            // input_ = new rfans_driver::InputPCAP(this, config_.dataport, packet_rate, config_.simu_filepath, config_.device_ip,false,false,0.0);
            input_ = new rfans_driver::InputPCAP(nh, config_.dataport, packet_rate, config_.simu_filepath, config_.device_ip,false,false,0.0);
        } else {
            m_devapi = new rfans_driver::IOSocketAPI(config_.device_ip, config_.dataport, config_.dataport);
            configDeviceParams();
        }

        // s_this = this;
    }
    
    virtual ~RfansDriverNode()
    {
        if (m_devapi) {
            delete m_devapi;
        }

        if (input_) {
            delete input_;
        }
    }

    /** @brief Rfnas Driver Core */
    bool spinOnce()
    {
        if (worRealtime()) {
            return spinOnceRealtime();
            RCLCPP_INFO(this->get_logger(),"real");
        } 
        else {
            return spinOnceSimu();
            RCLCPP_INFO(this->get_logger(),"simu");
        }
        RCLCPP_INFO(this->get_logger(),"kani");
    }

    int spinOnceRealtime()
    {
        int rtn = 0;

        m_devapi->revPacket(tmpPacket);

        rtn = m_devapi->getPacket(tmpPacket);
        if (rtn > 0) {
            m_output->publish(tmpPacket);
        }
        
        return rtn;
    }

    bool spinOnceSimu()
    {
    // rfans_driver_msgs::msg::RfansScan scan(new rfans_driver_msgs::msg::RfansScan);
    rfans_driver_msgs::msg::RfansScan scan;
    // auto scan = std::make_shared<rfans_driver_msgs::msg::RfansScan>();
    rfans_driver_msgs::msg::RfansPacket pkt;
    scan.packets.resize(32);
    for (int i = 0; i < 32; i++) {
        while (true) {
            int rc = input_->getPacket(scan.packets[i]);
            if(rc ==0)break;
            if(rc <0) return false;
        }
    }

    RCLCPP_INFO(this->get_logger(),"Publishing a full Rfans scan%s","");
    scan.header.stamp = scan.packets.back().stamp;
    scan.header.frame_id = "world";
    pkt.data.resize(scan.packets.size()*packet_size_pcap);
    for(int j=0; j<32; j++ )
    {
        memcpy(&pkt.data[j*packet_size_pcap],&(scan.packets[j].data[0]),packet_size_pcap);
    }
    pkt.stamp = scan.packets.back().stamp;
    pkt.udpcount = scan.packets.size();
    //pkt.udpSize = sizeof(rfans_driver::Packet().data);
    pkt.udpsize = packet_size_pcap;
    m_output->publish(pkt);
    memset(&pkt,0,sizeof(pkt));
    // return true;

        return true;
    }

private:
//     bool CommandHandle(const std::shared_ptr<rfans_driver_msgs::srv::RfansCommand::Request> req,
//                    std::shared_ptr<rfans_driver_msgs::srv::RfansCommand::Response> res)
// {
//     res->status = 1;

//     RCLCPP_INFO(rclcpp::get_logger("rfans_driver"), "request: cmd=%d, speed=%d Hz", req->cmd, req->speed);
//     RCLCPP_INFO(rclcpp::get_logger("rfans_driver"), "sending back response: [%d]", res->status);

//     DEB_PROGRM_S tmpProg;
//     tmpProg.cmdstat = static_cast<DEB_CMD_E>(req->cmd);
//     tmpProg.dataFormat = eFormatCalcData;
//     tmpProg.scnSpeed = req->speed;

//     if (s_this) {
//         s_this->prog_Set(tmpProg);
//     }

//     return true;
// }

    int prog_Set(DEB_PROGRM_S &program)
    {
        unsigned int tmpData = 0;

        switch (program.dataFormat) {
        case eFormatCalcData:
            tmpData |= CMD_CALC_DATA;
            break;
        case eFormatDebugData:
            tmpData |= CMD_DEBUG_DATA;
            break;
        }
        m_devapi->HW_WRREG(0, REG_DATA_TRANS, tmpData);
        //===============================================================
        tmpData = 0;
        switch (program.scnSpeed) {
        case ANGLE_SPEED_10HZ:
            tmpData |= CMD_SCAN_ENABLE;
            tmpData |= CMD_SCAN_SPEED_10HZ;
            break;
        case ANGLE_SPEED_20HZ:
            tmpData |= CMD_SCAN_ENABLE;
            tmpData |= CMD_SCAN_SPEED_20HZ;
            break;
        case ANGLE_SPEED_5HZ:
            tmpData |= CMD_SCAN_ENABLE;
            tmpData |= CMD_SCAN_SPEED_5HZ;
            break;
        default:
            tmpData |= CMD_SCAN_ENABLE;
            tmpData |= CMD_SCAN_SPEED_5HZ;
            break;
        }

        tmpData |= CMD_LASER_ENABLE;
        switch (program.cmdstat) {
        case eDevCmdWork:
            m_devapi->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
            break;
        case eDevCmdIdle:
            tmpData = CMD_RCV_CLOSE;
            m_devapi->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
            break;
        case eDevCmdAsk:
            break;
        default:
            break;
        }

        return 0;

    }
    int datalevel_Set(DEB_PROGRM_S &program)
    {
        unsigned int regData =0;
        switch (program.dataFormat) {
        case eFormatCalcData:
            regData |= CMD_CALC_DATA;
            break;
        case eFormatDebugData:
            regData |= CMD_DEBUG_DATA;
            break;
        }
        m_devapi->HW_WRREG(0, REG_DATA_TRANS, regData);
        regData =0;
        switch (program.dataLevel) {
        case LEVEL0_ECHO:
            regData = CMD_LEVEL0_ECHO;
            break;
        case LEVEL0_DUAL_ECHO:
            regData= CMD_LEVLE0_DUAL_ECHO;
            break;
        case LEVEL1_ECHO:
            regData = CMD_LEVEL1_ECHO;
            break;
        case LEVEL1_DUAL_ECHO:
            regData = CMD_LEVEL1_DUAL_ECHO;
            break;
        case LEVEL2_ECHO:
            regData = CMD_LEVEL2_ECHO;
            break;
        case LEVEL2_DUAL_ECHO:
            regData = CMD_LEVEL2_DUAL_ECHO;
            break;
        case LEVEL3_ECHO:
            regData = CMD_LEVEL3_ECHO;
            break;
        case LEVEL3_DUAL_ECHO:
            regData = CMD_LEVEL3_DUAL_ECHO;
            break;
        default:
            break;
        }

        switch (program.cmdstat) {
        case eDevCmdWork:
            m_devapi->HW_WRREG(0, REG_DATA_LEVEL, regData);
            break;
        case eDevCmdAsk:
            break;
        default:
            break;
        }

        return 0;

    }
    void setupNodeParams()
    {
        this->declare_parameter<std::string>("model", "R-Fans-16");
        this->declare_parameter<std::string>("advertise_name", "rfans_packets");
        this->declare_parameter<std::string>("control_name", "rfans_control");
        this->declare_parameter<int>("device_port", 2014);
        this->declare_parameter<std::string>("device_ip", "192.168.0.3");
        this->declare_parameter<int>("rps", 10);
        this->declare_parameter<std::string>("pcap", "");
        this->declare_parameter<int>("data_level",3);

        this->get_parameter("model", config_.device_name);
        this->get_parameter("advertise_name", config_.advertise_path);
        this->get_parameter("control_name", config_.command_path);
        this->get_parameter("device_port", config_.dataport);
        this->get_parameter("device_ip", config_.device_ip);
        this->get_parameter("rps", config_.scnSpeed);
        this->get_parameter("pcap", config_.simu_filepath);
        this->get_parameter("data_level",config_.data_level);
        RCLCPP_INFO(this->get_logger(),"Setup Node Param!!!");
    }

    bool worRealtime()
    {
        return (config_.simu_filepath.empty());
        
        // return ((config_.simu_filepath == "")? true : false);
    }

    double calcReplayPacketRate()
    {
      double rate = 0.0f;
    std::string device = config_.device_name;
    int data_level = config_.data_level;
    bool dual_echo = config_.dual_echo;
    RCLCPP_INFO(rclcpp::get_logger("rfans_driver"), "Device: %s",device.c_str());
    //one second generate 640k points,and each packet have 32*12 points.
    if (device == "R-Fans-32") {
        if (((data_level == 0) || (data_level == 1)) && dual_echo) {
            rate = 6666.67;
        } else if (((data_level == 0)|| (data_level == 1)) && (!dual_echo)) {
            rate = 3333.33;
        } else if (data_level == 2 && dual_echo) {
            rate = 4000;
        } else if (data_level == 2 &&(!dual_echo)) {
            rate = 2000;
        } else if (data_level == 3 && dual_echo) {
            rate = 3333.33;
        } else if (data_level == 3 &&(!dual_echo)) {
            rate = 1666.67;
        } else if (data_level == 4 && dual_echo) {//FIXME: should check GM & BK format
            rate = 3333.33;
        }
        else{
            rate = 1666.67;
        }
    } else if (device == "R-Fans-16") {
        if (((data_level == 0)|| (data_level == 1)) && dual_echo) {
            rate = 3333.33;
        } else if (((data_level == 0)|| (data_level == 1)) && (!dual_echo)) {
            rate = 1666.67;
        } else if (data_level == 2 && dual_echo) {
            rate = 2000;
        } else if (data_level == 2 && (!dual_echo)) {
            rate = 1000;
        } else if (data_level == 3 && dual_echo) {
            rate = 1666.67;
        } else if (data_level == 3 && (!dual_echo)) {
            rate = 833.3;
        } else if (data_level == 4) {
            rate = 833.3;
        }
    }else if(device == "R-Fans-V6K"){
        if (data_level == 2 && dual_echo) {
            rate = 2133.33;
        } else if (data_level == 2 && (!dual_echo)) {
            rate = 1066.67;
        } else if (data_level == 3 && dual_echo) {
            rate = 1562.5;
        } else if (data_level == 3 && (!dual_echo)) {
            rate = 781.25;
        }
    }
    else if (device == "C-Fans-128") {
        rate = 1666.67;
        // TODO:
    } else if (device == "C-Fans-32") {
        // TODO:
        rate = 416.67;
    } else {
        //rate = 1666.67;
        rate = 781.25;
    }




        return rate;
    }


    void configDeviceParams()
    {
        int data_level = config_.data_level;
    bool dual_echo = config_.dual_echo;

    DEB_PROGRM_S params;
    params.cmdstat = eDevCmdWork;
    params.dataFormat = eFormatCalcData;

    // set start rps
    params.scnSpeed =  config_.scnSpeed;
    prog_Set(params);

    if (data_level== 0 && !dual_echo) {
        params.dataLevel = LEVEL0_ECHO;
    } else if (data_level == 0 && dual_echo) {
        params.dataLevel = LEVEL0_DUAL_ECHO;
    } else if (data_level == 1 && !dual_echo) {
        params.dataLevel = LEVEL1_ECHO;
    } else if (data_level == 1 && dual_echo) {
        params.dataLevel = LEVEL1_DUAL_ECHO;
    } else if(data_level == 2 && !dual_echo) {
        params.dataLevel = LEVEL2_ECHO;
    } else if (data_level == 2 && dual_echo) {
        params.dataLevel = LEVEL2_DUAL_ECHO;
    } else if (data_level == 3 && !dual_echo) {
        params.dataLevel = LEVEL3_ECHO;
    } else {
        params.dataLevel = LEVEL3_DUAL_ECHO;
    }

    // set data level
    datalevel_Set(params);
    }

    rfans_driver_msgs::msg::RfansPacket tmpPacket;
    rclcpp::Service<rfans_driver_msgs::srv::RfansCommand>::SharedPtr server_;
    rclcpp::Publisher<rfans_driver_msgs::msg::RfansPacket>::SharedPtr m_output;
    rfans_driver::InputPCAP* input_;
    rfans_driver::IOSocketAPI* m_devapi;
    DEB_PROGRM_S params;
    std::string device;
    int data_level;
    bool dual_echo;
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
    // ... この後のメンバ変数も省略 ...
};

} // rfans_driver namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rfans_driver::RfansDriverNode>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    // while (rclcpp::ok()&&node->spinOnce())

    while (rclcpp::ok())
    {
        node->spinOnce();
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}

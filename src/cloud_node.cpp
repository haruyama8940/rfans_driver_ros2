#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <string>
#include "rfans_driver_msgs/srv/rfans_command.hpp"
#include "rfans_driver/bufferDecode.hpp"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <unistd.h>


static const int RFANS_POINT_CLOUD_NUM = 1024 ;

static std::vector<SCDRFANS_BLOCK_S> outBlocks ;
static std::vector<RFANS_XYZ_S> outXyzBlocks ;
static sensor_msgs::msg::PointCloud2 outCloud ;

static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr s_output;

static int scanSpeed;
extern int gs_pointsPerRound;
extern bool use_gps;
extern int year;
extern int month;
extern int day;
extern int hour;
extern double min_range;
extern double max_range;
extern double min_angle;
extern double max_angle;
extern int ringID;
extern bool use_laserSelection_;
typedef int ( *PFUNC_THREAD)(void *);
static DEVICE_TYPE_E s_deviceType = DEVICE_TYPE_NONE;
typedef struct{
    unsigned int pkgflag;
    unsigned int pkgnumber;
    unsigned int date;
    unsigned short time;
    unsigned int maca;
    unsigned short macb;
    unsigned short dataport;
    unsigned short msgport;
    unsigned char motorspd;
    unsigned int deviceType;
    unsigned short phaseAngle;
    unsigned char padding[225];
}RFANS_HEARTBEAT_S;
static tm gs_lidar_time;

static  pthread_t ssCreateThread(int pri, void * obj, PFUNC_THREAD fnth) {
    pthread_t thrd_ ;
    pthread_create(&thrd_, NULL, (void *(*)(void*))fnth, (void *)obj);
    return thrd_ ;
}
static int heartbeat_thread_run(void *para) {
    printf("heart beat thread start \n");
    // create socket, then read broadcast port2030 package
    int client_fd;
    int rtn;
    struct sockaddr_in ser_addr;
    bool optval = true;
    const int opt = -1;
    client_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (client_fd < 0) {
        printf("create socket failed!\n");
        return -1;
    }

    // memset(&ser_addr, 0, sizeof(ser_addr));
    bzero(&ser_addr,sizeof(struct sockaddr_in));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_port = htons(2030);
    ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    rtn =setsockopt(client_fd,SOL_SOCKET,SO_REUSEPORT,(char*)&opt,sizeof(opt));
    if(rtn<0)
    {
        printf("setsockopt failed! \n");
    }
    if (bind(client_fd,(struct sockaddr*)&ser_addr, sizeof(sockaddr_in)) < 0) {
        printf("bind server failed!\n");
        return -1;
    }
    socklen_t len;
    struct sockaddr_in src;
    printf("after heartbeat init \n");
    while(1) {
        unsigned char buff[512] = {'\0'};
        int rcv = recvfrom(client_fd, buff, sizeof(buff), 0, (struct sockaddr*)&ser_addr, &len);
        if (rcv > 0) {
            RFANS_HEARTBEAT_S hb;
            memset(&hb, '\0', sizeof(RFANS_HEARTBEAT_S));
            memcpy(&hb, buff, sizeof(RFANS_HEARTBEAT_S));
            swapchar((unsigned char*)&hb.pkgflag, sizeof(hb.pkgflag));
            swapchar((unsigned char*)&hb.pkgnumber, sizeof(hb.pkgnumber));
            swapchar((unsigned char*)&hb.date, sizeof(hb.date));
            swapchar((unsigned char*)&hb.time, sizeof(hb.time));

            if (hb.pkgflag == 0xe1e2e3e4) {//heartbeat flag
                gs_lidar_time.tm_year = ((hb.date& 0xFF000000)>>24)+2000;;
                gs_lidar_time.tm_mon =((hb.date & 0xFF0000) >> 16);
                gs_lidar_time.tm_mday = ((hb.date & 0xFF00) >> 8);;
                gs_lidar_time.tm_hour = (hb.date & 0xFF);
                year = gs_lidar_time.tm_year;
                month = gs_lidar_time.tm_mon;
                day = gs_lidar_time.tm_mday;
                hour = gs_lidar_time.tm_hour;
//                ROS_INFO(" Dong %d,%d,%d,%d",year,month,day,hour);

//                printf("flag:0x%08x, num:0x%08x, date:0x%08x, time:0x%08x\n",
//                        hb.pkgflag,hb.pkgnumber, hb.date, hb.time);
            }
        }
        usleep(500000);//500ms;
    }

    close(client_fd);

    return 0;
}

class CalculationNode : public rclcpp::Node
{
public:
  CalculationNode() : Node("calculation_node")
  {
    calcurate_func();
    // s_sub_ = this->create_subscription<rfans_driver_msgs::msg::RfansPacket>("rfans_driver/rfans_packets",1,
    //         std::bind(&CalculationNode::RFansPacketReceived, this, std::placeholders::_1));
    // s_output_ =this->create_publisher<sensor_msgs::msg::PointCloud2>("rfans_driver/rfans_points",10);
    s_sub_ = this->create_subscription<rfans_driver_msgs::msg::RfansPacket>(advertise_path ,10,
            std::bind(&CalculationNode::RFansPacketReceived, this, std::placeholders::_1));
    s_output_ =this->create_publisher<sensor_msgs::msg::PointCloud2>(subscribe_path  ,10);
  }
    void RFansPacketReceived(const rfans_driver_msgs::msg::RfansPacket::SharedPtr pkt);
    void callback();
    void calcurate_func();
    void setparam();
private:
    rclcpp::Subscription<rfans_driver_msgs::msg::RfansPacket>::SharedPtr s_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr s_output_;

    std::string node_name =  rclcpp::Node::get_name();
    std::string frame_id_str = "/world";
    std::string frame_id_path = node_name + "/frame_id";
    std::string ip_str = std::string("rfans_driver/") + "device_ip";
    std::string data_level_param = std::string("rfans_driver/")+"data_level";

    std::string advertise_name = "rfans_points";
    std::string advertise_path = node_name + "/advertise_name";
    std::string subscribe_name = "rfans_packets";
    std::string subscribe_path = node_name + subscribe_name;

};


void CalculationNode::RFansPacketReceived(const rfans_driver_msgs::msg::RfansPacket::SharedPtr pkt) {
  int rtn = 0 ;
//   RCLCPP_INFO(this->get_logger(),"rfans packet");
  rtn =  SSBufferDec::Depacket(pkt, outCloud,s_output_, s_deviceType) ;
  return ;
}


void CalculationNode::callback(){
    min_range = this->declare_parameter("min_range", 0.0); //0.0~1.0[m]
    max_range = this->declare_parameter("max_range", 180.0); //0.1~200[m]
    min_angle = this->declare_parameter("min_angle", 0.0);//0.0~360[deg]
    max_angle = this->declare_parameter("max_angle", 360.0);//0.0~360[deg]
    use_laserSelection_ = this->declare_parameter("use_laserSelection", false);
    ringID = this->declare_parameter("ringID", 0);
    // min_range = config.min_range;
    // max_range = config.max_range;
    // min_angle = config.min_angle;
    // max_angle = config.max_angle;
    // use_laserSelection_ = config.use_laserSelection;
    // ringID = config.laserID;
}


void CalculationNode::calcurate_func(){
    bool use_gps_;
    bool use_double_echo_ = false;
    std::string node_name =  rclcpp::Node::get_name();
    /*
    std::string frame_id_str = "/surestar";
    std::string frame_id_path = node_name + "/frame_id";
    std::string ip_str = std::string("rfans_driver/") + "device_ip";
    std::string data_level_param = std::string("rfans_driver/")+"data_level";
    */
    this->declare_parameter("frame_id","world");
    this->get_parameter(frame_id_path,frame_id_str);

    // this->get_parameter(ip_str,ip_address);
    // this->get_parameter(data_level_param,data_level_);
    // this->get_parameter("model",device_type);
    SSBufferDec::InitPointcloud2(outCloud,frame_id_str);

    //advertise name
    /*
    std::string advertise_name = "rfans_points";
    std::string advertise_path = node_name + "/advertise_name";
    */
    this -> declare_parameter(advertise_name,"rfans_poins");
    this -> get_parameter(advertise_path,advertise_name);
    advertise_path = "rfans_driver/" + advertise_name;

    //subscribe name
    /*
    std::string subscribe_name = "rfans_packets";
    std::string subscribe_path = node_name + "/subscribe_name";
    */
    this->declare_parameter(subscribe_name,subscribe_path);
    this->get_parameter(subscribe_path, subscribe_name);
    subscribe_path = "rfans_driver/" + subscribe_name;
   
    RCLCPP_INFO(this->get_logger(),"%s : subscribe name %s : %s",node_name.c_str(), subscribe_name.c_str(), subscribe_path.c_str() );
    pthread_t s_heartbeat_worker_id = ssCreateThread(1, NULL, heartbeat_thread_run) ;

    //angle duration
    double angle_duration;
    angle_duration = this->declare_parameter("angle_duration", 360.0);
    SSBufferDec::SetAngleDuration(angle_duration);
    RCLCPP_INFO(this->get_logger(),"%s : angle_duration : %f",node_name.c_str(), angle_duration);
    
    //rfans_point sub
    // rclcpp::Subscription<rfans_driver_msgs::msge::RfansPacket>::SharedPtr s_sub_;
    //point_cloud pub
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr s_output_;
    this->declare_parameter("rfans_driver/rps",scanSpeed);
    this->declare_parameter("rfans.use_double_echo",use_double_echo_);
    bool ok = this->get_parameter("/rfans_driver/rps",scanSpeed);
    bool ok1 = this->get_parameter("rfans_driver.use_double_echo", use_double_echo_);
    // bool ok= ros::param::get("/rfans_driver/rps",scanSpeed);
    // bool ok1 = ros::param::get("/rfans_driver/use_double_echo",use_double_echo_);

    if(ok && ok1){
        if(scanSpeed ==5)
        {
            if(use_double_echo_) {
                gs_pointsPerRound = 8000;
            }
            else {
                gs_pointsPerRound = 4000;
            }
        }else if(scanSpeed == 10)
        {
            if (use_double_echo_) {
                gs_pointsPerRound = 4000;
            }
            else {
                gs_pointsPerRound = 2000;
            }
        }
        else
        {
            if(use_double_echo_) {
                gs_pointsPerRound = 2000;
            }
            else {
                gs_pointsPerRound = 1000;
            }
        }
    }

    std::string device_model;
    // ros::param::get("model",device_model);
    this->declare_parameter("model",device_model);
    this->get_parameter("model",device_model);
    RCLCPP_INFO(this->get_logger(),"device_model_value: %s", device_model.c_str());
    if (device_model == "C-Fans-128")
    {
        s_deviceType = DEVICE_TYPE_CFANS;
        std::string revise_angle_key = node_name + "/revise_angle_128";
        std::string revise_angle_value = "0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0,";//default
        this->declare_parameter(revise_angle_key,revise_angle_value);
        this->get_parameter(revise_angle_key, revise_angle_value);
        initCFansPara(revise_angle_value);
    }
    else if(device_model == "C-Fans-32")
    {
        s_deviceType = DEVICE_TYPE_CFANS;
        std::string revise_angle_key = node_name + "/revise_angle_32";
        std::string revise_angle_value = "0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0,";//default
        this->declare_parameter(revise_angle_key, revise_angle_value);
        this->get_parameter(revise_angle_key, revise_angle_value);
        initCFans_32(revise_angle_value);
    }
    else
    {
        s_deviceType = DEVICE_TYPE_RFANS;
    }

    std::string save_xyz_key = node_name + "/save_xyz";
    std::string save_xyz_value = "no";//default not save
    this->declare_parameter(save_xyz_key,save_xyz_value);
    this->get_parameter(save_xyz_key, save_xyz_value);
    if (0 == strcmp(save_xyz_value.c_str(), "yes")) {
        SSBufferDec::setSaveXYZ(true);
    } else {
        SSBufferDec::setSaveXYZ(false);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalculationNode>();
    node->calcurate_func();
    node->callback();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }
    
    rclcpp::shutdown();
    return 0;
}

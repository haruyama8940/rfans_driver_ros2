#include "rfans_driver/bufferDecode.hpp"
#include <string.h>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
// #include <ros/ros.h>
#include <sys/time.h>
#include "rfans_driver/ioapi.hpp"
#include <ctime>


static const int LINE_POINT_MINI_COUNT = 500; //一条扫描线最少点的个数
static const float ANGLE_CIRCLE_CONDITION = 270; //角度抖动处理值
static const float UINTCONVERT = 0.01;

static const size_t packet_size = sizeof(rfans_driver_msgs::msg::RfansPacket().data);
static const size_t BLOCK_COUNT_MAX = sizeof(rfans_driver_msgs::msg::RfansPacket().data)/138;

static float s_angle_duration = 359 ;
const double TIME_FLAG_SCALE = 0.0000001;
static int LINE_POINT_COUNT = 256*1024;
static std::vector<RFANS_XYZ_S> s_lineData;
static std::vector<RFANS_XYZ_S> s_rowData[32];
static std::vector<RFANS_XYZ_S> s_tempData[32];
static int point_count=0;
static int s_lineCount = 0 ;
static float s_lastAngle = 0 ;
static bool s_findStartZero = false;
static bool s_zeroFrame = false;
int gs_pointsPerRound = 2000;
bool use_gps;
int data_level_ = 3;
static std::vector<float> s_vangles;
static std::vector<float> s_hangles;
//static std::vector<float> s_reviseangles;

static DEVICE_TYPE_E s_device_type;
FILE *g_xyzFile = NULL;
static int roundCount;
static unsigned int gpsTime ;
std::string ip_address;
int year;
int month;
int day;
int hour;
double min_range;
double max_range;
double min_angle;
double max_angle;
int ringID;
bool use_laserSelection_;
std::string device_type;

inline tm stampTransform(int year_,int month_,int day_,int hour_)
{
    tm time_c;
    time_c.tm_year = year_-1900;
    time_c.tm_mon = month_;
    time_c.tm_mday = day_+1;
    time_c.tm_hour = hour_;
    return time_c;

}

inline bool recordAsPPSAngel(unsigned int lastTime, unsigned int curTime) {
    bool recordPPSAngle = false;

    float last = lastTime * 0.000001;
    float cur = curTime * 0.000001;

    unsigned int lastZS = (unsigned int)last;
    float lastXS = last - (float)lastZS;

    unsigned int curZS = (unsigned int)cur;
    float curXS = cur - (float)curZS;

    float diffXS = curXS - lastXS;
    if (diffXS <= -0.900000) {
        recordPPSAngle = true;
    }
    return recordPPSAngle;
}


inline bool sortByHangle(const RFANS_XYZ_S &v1, const RFANS_XYZ_S &v2) {
    return v1.hangle < v2.hangle;
}

inline int checkFrame_sum(unsigned char flag,float mtAngle,int mtlaserId,sensor_msgs::msg::PointCloud2 &outCloud) {
    int rtn = 0 ;
    static float s_angleSum = 0;
    float angleDif = 0 ;
    if(mtlaserId!=0) return rtn ;
    int num_lasers =0;
    if(device_type == "R-Fans-16" || device_type == "R-Fans-V6K"){
        num_lasers = 16;
    }
    else{
        num_lasers = 32;
    }
    int angleZero = (int)mtAngle;
    if (s_findStartZero == false) {
        if (angleZero == 0) {
            s_lineCount = 0 ;
            s_angleSum = 0 ;
            for (int i = 0; i < num_lasers; ++i) {
                s_rowData[i].clear();
            }
            s_findStartZero = true;
        }
        return rtn;
    }


    if(mtAngle > s_lastAngle) {
        angleDif = mtAngle-s_lastAngle;
        s_angleSum += angleDif;
        s_lastAngle = mtAngle;
    } else {
        //1: 360-> 0
        if(s_lastAngle -mtAngle> 300)  {
            angleDif = 360 - s_lastAngle+mtAngle;
            s_angleSum += angleDif;
            s_lastAngle = mtAngle;
        } else {
            //nothing
        }

    }

    if(s_angleSum >= s_angle_duration) {
        timeval tval_begin, tval_end;
        gettimeofday(&tval_begin, NULL);
        int pointCnt = 0;
        //first cycle.
        if(s_zeroFrame == false)
        {
            for(int i=0; i< num_lasers;i++)
            {
                int lsrPointCnt = (s_rowData[i]).size();
                if(lsrPointCnt>0)
                {
                    s_tempData[i] = s_rowData[i];
                    s_rowData[i].clear();

                }
            }
            s_zeroFrame= true;
            s_lineCount = 0 ;
            s_angleSum = 0 ;
            return rtn;
        }

        int pointTotalCnt=0;
        //next cycle
        for(int i=0;i< num_lasers;i++)
        {
            int count = (s_rowData[i]).size();
            if(count>0)
            {


                for(int k=0; k< (s_tempData[i]).size();) {
                    if((s_tempData[i])[k].hangle>345 && k==0)
                    {
                        s_tempData[i].erase((s_tempData[i]).begin()+k);
                        k = 0;
                        continue;
                    }
                    k++;
                }

                bool findNextFrameZero = false;
                for (int idx = 0; idx < (s_rowData[i]).size(); ++idx) {
                    if((s_rowData[i])[idx].hangle>345 && findNextFrameZero == false)
                    {

                        RFANS_XYZ_S temxyz = (s_rowData[i])[idx];
                        s_tempData[i].push_back(temxyz);
                    } else {
                        findNextFrameZero = true;
                    }
                }
//                ROS_INFO("gs_pointsPerRound: %d",gs_pointsPerRound);
                if (s_tempData[i].size() <= gs_pointsPerRound) {
                    for(int index=s_tempData[i].size(); index<gs_pointsPerRound;index++)
                    {
                        RFANS_XYZ_S temxyz;
                        temxyz.x=NAN;
                        temxyz.y=NAN;
                        temxyz.z=NAN;
                        temxyz.intent=0.0;
                        temxyz.laserid=i;
                        temxyz.timeflag=0.0;
                        temxyz.hangle=360.0;
                        s_tempData[i].push_back(temxyz);

                    }
                }
                else{
                    s_tempData[i].erase(s_tempData[i].begin()+gs_pointsPerRound,s_tempData[i].end());

                }
                //else gs_pointPerRound >2000.
                for(std::vector<RFANS_XYZ_S>::iterator it =s_tempData[i].begin();it!=s_tempData[i].end();it++)
                {
                    if(flag==RFANS_PRODUCT_MODEL_CFANS_X32_0X80){
                      double idMirror[4]={0,1,2,3};
                      (*it).laserid = (*it).laserid*4+idMirror[(*it).mirrorid];
                    }
                    s_lineData[pointCnt++] = *it;
                }
            }
            pointTotalCnt+=s_tempData[i].size();
        }
        for (int i = 0; i <pointTotalCnt; i++) {
            if(g_xyzFile) {

                //                   fprintf(g_xyzFile,"%d,%f\r\n",s_lineData[i].laserid,s_lineData[i].hangle);
                fprintf(g_xyzFile,"%d,%f,%f,,%f,%f,%d\r\n",s_lineData[i].laserid,s_lineData[i].x,s_lineData[i].y,s_lineData[i].z, s_lineData[i].hangle,s_lineData[i].mirrorid);
                //                   fprintf(g_xyzFile,"%d,%f,%f,%f,%f \r\n",
                //                                     s_lineData[i].laserid, s_lineData[i].x, s_lineData[i].y, s_lineData[i].z, s_lineData[i].hangle);
                fflush(g_xyzFile);
            }
        }
        gettimeofday(&tval_end, NULL);
        int elapse = (tval_end.tv_sec - tval_begin.tv_sec)*1000000 + (tval_end.tv_usec - tval_begin.tv_usec);
        if (g_xyzFile)
        {
            fprintf(g_xyzFile,"time elapsed:%d us \r\n", elapse);
            fflush(g_xyzFile);
        }

        if(use_gps)
        {
            unsigned int time = gpsTime;
            tm time_c;
            // time_c = stampTransform(year,month,day,hour);
            // time_c.tm_min = (int)(time/60000000);
            // time_c.tm_sec = (int)(time/1000000-time_c.tm_min*60);
            // time_t  timethen = mktime(&time_c);
            // ros::Time rosTime;
            // rosTime.sec = (unsigned int)timethen;
            // rosTime.nsec = (unsigned int)(time%1000000)*1000;
            // outCloud.header.stamp = rosTime;
        }
        else{
            rclcpp::Time current_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            outCloud.header.stamp = current_stamp;
            // outCloud.header.stamp = ros::Time::now();
        }
        outCloud.width = pointTotalCnt;
        outCloud.data.resize( outCloud.point_step*outCloud.width);
        outCloud.row_step = outCloud.data.size();
        memcpy(&outCloud.data[0] , &s_lineData[0], outCloud.data.size() );
        rtn = 1;
        s_lineCount = 0 ;
        s_angleSum = 0 ;
        for (int i = 0; i < num_lasers; ++i) {
            s_tempData[i].clear();
        }
        for(int i=0; i< num_lasers;i++)
        {
            s_tempData[i]=s_rowData[i];
            s_rowData[i].clear();
        }
    }
    return rtn ;

}


inline int checkOneRound(float mtAngle,int mtlaserId,sensor_msgs::msg::PointCloud2 &outCloud) {
  int rtn = 0 ;
//  point_count++;
  static float s_angleSum = 0;
  float angleDif = 0 ;
  if(mtlaserId!=0) return rtn ;
  if(mtAngle > s_lastAngle) {
    angleDif = mtAngle-s_lastAngle;
    s_angleSum += angleDif;
    s_lastAngle = mtAngle;
  } else {
    //1: 360-> 0
    if(s_lastAngle -mtAngle> 300)  {
      angleDif = 360 - s_lastAngle+mtAngle;
      s_angleSum += angleDif;
      s_lastAngle = mtAngle;
    } else {
      //nothing
    }

  }

  if(s_angleSum >= s_angle_duration) {
//       timeval tval_begin;
//      gettimeofday(&tval_begin, NULL);
//        printf("s: %ld, us: %06ld\n", tval_begin.tv_sec, tval_begin.tv_usec);
      if(use_gps)
    {
        unsigned int time = gpsTime;
        tm time_c;
        // time_c = stampTransform(year,month,day,hour);
        // time_c.tm_min = (int)(time/60000000);
        // time_c.tm_sec = (int)(time/1000000-time_c.tm_min*60);
        // time_t  timethen = mktime(&time_c);
        // ros::Time rosTime;
        // rosTime.sec = (unsigned int)timethen;
        // rosTime.nsec = (unsigned int)(time%1000000)*1000;
    }
    else
    {
        rclcpp::Time current_stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        outCloud.header.stamp = current_stamp;
        // outCloud.header.stamp = ros::Time::now();
    }
    outCloud.width = s_lineCount;
    outCloud.data.resize( outCloud.point_step*outCloud.width  );
    outCloud.row_step = outCloud.data.size();
    memcpy(&outCloud.data[0] , &s_lineData[0], outCloud.data.size() );
    rtn = 1;
    s_lineCount = 0 ;
    s_angleSum = 0 ;
  }

  return rtn ;
}


inline int checkFrame(float mtAngle,int mtlaserId,sensor_msgs::msg::PointCloud2 &outCloud) {
    int rtn = 0 ;
    float angleDif = 0 ;
    if( mtlaserId !=0 ) return rtn ;

    if(mtAngle > s_lastAngle) {
        angleDif = mtAngle-s_lastAngle;
    } else {
        angleDif = 360 - s_lastAngle+mtAngle;
    }

    if(angleDif < 0 ) angleDif= 360+angleDif;

    if(angleDif >= s_angle_duration) {
        outCloud.width = s_lineCount;
        outCloud.data.resize( outCloud.point_step*outCloud.width  );
        outCloud.row_step = outCloud.data.size();
        memcpy(&outCloud.data[0] , &s_lineData[0], outCloud.data.size() );
        rtn = 1;
        s_lastAngle = mtAngle;
        s_lineCount = 0 ;

    }
    return rtn ;
}
inline int calCFansMirrorIndex(int angle_fix,int laserID,float &range){
    int mirrorIndex = 0;
    //修正零位角度 （奇偶路相差60°）
    switch (laserID % 4) {
    case 0:
        angle_fix  += m_anglePara[24];
        break;
    case 1:
        angle_fix  += m_anglePara[25];
        break;
    case 2:
        angle_fix  += m_anglePara[26];
        break;
    case 3:
        angle_fix  += m_anglePara[27];
        break;
    }

    if (angle_fix< 0)
        angle_fix += 360;
    else if (angle_fix >360)
        angle_fix -= 360;

    //根据点的角度范围和激光器的奇偶路，确定镜面的编号
    if (laserID % 2 == 1) {
        //if (angle_fix >= 0 && angle_fix  < 90){
        if ((angle_fix >= 15 ) && (angle_fix < 90 ))  {
            mirrorIndex = 2;
        }
        //else if (angle_fix >= 90 && angle_fix  < 180) {
        else if ((angle_fix >= 105 ) && (angle_fix < 180 )) {
            mirrorIndex = 3;
        }
        // else if (angle_fix >= 180 && angle_fix  < 270) {
        else if ((angle_fix >= 195 ) && (angle_fix < 270 )) {
            mirrorIndex = 0;
        }
        //else if (angle_fix >= 270 && angle_fix  < 360){
        else if ((angle_fix >= 285 ) && (angle_fix < 360 )) {
            mirrorIndex = 1;
        }
        else {
            range = 0;
        }

    }
    else{
        //if ((angle_fix >= 0 && angle_fix  < 90)) {
        if ((angle_fix >= 0) && (angle_fix < 75 )) {
            mirrorIndex = 3;
        }
        //else if (angle_fix >= 90 && angle_fix  < 180) {
        else if ((angle_fix >= 90 ) && (angle_fix < 165 )) {
            mirrorIndex = 0;
        }
        //else if (angle_fix >= 180 && angle_fix  < 270) {
        else if ((angle_fix >= 180 ) && (angle_fix < 255 )) {
            mirrorIndex = 1;
        }
        //else if (angle_fix >= 270 && angle_fix  < 360) {
        else if ((angle_fix >= 270 ) && (angle_fix < 345 )) {
            mirrorIndex = 2;
        }
        else {
            range = 0;
        }
    }
    return mirrorIndex;
}


//process level 0 data.
inline int processPacketOri(PACKET_ORI_S* packet,sensor_msgs::msg::PointCloud2 &outCloud)
{
    int rtn =0;
    RFANS_XYZ_S tmpXyz;
    const float CONVERT_4mm_2m =0.004f;
    float tmpAngle, tmpRange;
    int rising_edge;
    int pulse_width;
    double timeOffset = 1.5625;
    float angular_z = 0.0018;//deg/us
    float s_angleStep;
//    unsigned char DEVICE_ID = (packet->factory>>8)&0xff;
    unsigned char DEVICE_ID = packet->gmReservedA;
    gpsTime = packet->gps_timestamp;
//    int tmpDif = packet->groups[1].azimuth_angle - packet->groups[0].azimuth_angle;
//      if ( tmpDif< -35000 ){
//        tmpDif = tmpDif + 36000;
//      }
//      s_angleStep = (tmpDif) / 32.0;
    for(int i =0; i < GROUP_NUM_ORI; i++)
    {
        GROUP_ORI_S* mtBlock = &packet->groups[i];
        if(i <=4)
        {
            int tmpAngleDiff = packet->groups[i+1].azimuth_angle - packet->groups[i].azimuth_angle;
            if (tmpAngleDiff < -35000)
            {
                tmpAngleDiff += 36000;
            }

            if(DEVICE_ID==0x4F||DEVICE_ID==0x55||DEVICE_ID==0x56||DEVICE_ID==0x57||DEVICE_ID==0x58)
            {
                angular_z = tmpAngleDiff*UINTCONVERT/(32*3.125);
            }

            else
            {
                angular_z = tmpAngleDiff*UINTCONVERT/(32*1.5625);
            }

        }

        else
        {
            int angle_sub = packet->groups[5].azimuth_angle - packet->groups[4].azimuth_angle;
            if(angle_sub < -35000)
            {
                angle_sub += 36000;
            }
            if(DEVICE_ID==0x4F||DEVICE_ID==0x55||DEVICE_ID==0x56||DEVICE_ID==0x57||DEVICE_ID==0x58)
            {
                angular_z = angle_sub*UINTCONVERT/(32*3.125);
            }

            else
            {
                angular_z = angle_sub*UINTCONVERT/(32*1.5625);
            }
        }

        for(int j=0; j< POINT_NUM_ORI; j++)
        {
            switch (DEVICE_ID) {
            case RFANS_PRODUCT_MODEL_V6A_X16M_0X4F:
            case RFANS_PRODUCT_MODEL_V6A_E1_0X55:
            case RFANS_PRODUCT_MODEL_V6A_E2_0X56:
                timeOffset =3.125;
                if(j<=15)
                {
                    tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j];//us
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6A_X16M_0X4F_0x55_0x56[j];//deg
                }
                else{
                    tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-16]+50;
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-16]+50);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6BC_16G_0X57:
            case RFANS_PRODUCT_MODEL_V6BC_16M_0X58:
                timeOffset =3.125;
                if(i<=15)
                {
                    tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6BC_16GM_0x57_0x58[j];
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6BC_16GM_0x57_0x58[j];
                }
                else{
                    tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6BC_16GM_0x57_0x58[j-16]+50;
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6BC_16GM_0x57_0x58[j-16]+50);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6G_X32_0X33:
            case RFANS_PRODUCT_MODEL_V6B_X32_0X50:
                timeOffset = 1.5625;
                tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6G_V6BC_X32_0x33_0x50[j];
                tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6G_V6BC_X32_0x33_0x50[j];
                break;
            case RFANS_PRODUCT_MODEL_V6_X32_0X40:
            case RFANS_PRODUCT_MODEL_V6_X16A_0X41:
            case RFANS_PRODUCT_MODEL_V6_X16B_0X42:
            case RFANS_PRODUCT_MODEL_V6_X16Even_0X43:
            case RFANS_PRODUCT_MODEL_V6_X16Odd_0X44:
            case RFANS_PRODUCT_MODEL_V6A_X32_0X4A:
            case RFANS_PRODUCT_MODEL_V6A_X16A_0X4B:
            case RFANS_PRODUCT_MODEL_V6A_X16B_0X4C:
            case RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D:
            case RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E:
                timeOffset = 1.5625;
                tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6_X32_0x40[j];
                tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6_X32_0x40[j];
                break;
            case RFANS_PRODUCT_MODEL_CFANS_X32_0X80:
                timeOffset = 1.5625;
                tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_CFANS_0x80[j];
                tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_CFANS_0x80[j];
                break;

            default:
                break;
            }

//            tmpXyz.timeflag = packet->gps_timestamp+timeOffset*i*j;
//            tmpAngle = (mtBlock->azimuth_angle+j*s_angleStep)*0.01;
            tmpRange = mtBlock->points[j].range*CONVERT_4mm_2m;
            rising_edge = mtBlock->points[j].rising_edge;
            pulse_width = ((mtBlock->points[i].intensity_pulse[1] & 0x0F) << 8)| mtBlock->points[i].intensity_pulse[2];
            tmpXyz.intent = (mtBlock->points[j].intensity_pulse[0]<<4)|((mtBlock->points[j].intensity_pulse[1]>>4)&0x0f);
            tmpXyz.laserid = j;
            tmpRange = mtBlock->points[j].range*CONVERT_4mm_2m;
            tmpXyz.mirrorid =0;
            calcXyz(DEVICE_ID,tmpRange,tmpAngle,tmpXyz);
            if(g_xyzFile) {
                fprintf(g_xyzFile,"%02d, %f, %f, %f\r\n",
                        tmpXyz.laserid,tmpXyz.x,tmpXyz.y,tmpXyz.z);
                fflush(g_xyzFile);
            }
            if(tmpRange > max_range || tmpRange < min_range|| tmpXyz.hangle>max_angle || tmpXyz.hangle<min_angle)
            {
                tmpXyz.x =NAN;
                tmpXyz.y = NAN;
                tmpXyz.z = NAN;
            }
            if(checkOneRound(tmpAngle,tmpXyz.laserid,outCloud)) rtn =1;
            s_lineData[s_lineCount] = tmpXyz;
            ++s_lineCount;
            if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1;

        }
        }
        return rtn;
}

//process level 2 data
inline int processPacketUser(PACKET_USER_S* packet, sensor_msgs::msg::PointCloud2 &outCloud)
{
    int rtn =0;
    RFANS_XYZ_S tmpXyz1;
    RFANS_XYZ_S tmpXyz2;
    const float CONVERT_4mm_2m =0.004f;
    uint8_t echo_flag = 1;
    float tmpAngle1, tmpRange1;
    float tmpAngle2, tmpRange2;
    double timeOffset = 1.5625;
    float angular_z;
    unsigned char DEVICE_ID =packet->gmReservedA;
    gpsTime = packet->gps_timestamp;
    for(int i =0; i < GROUP_NUM_USER; i++)//10 bolck
    {
        GROUP_USER_S* mtBlock = &packet->groups[i];

        int tmpAngleDiff = packet->groups[1].azimuth_angle - packet->groups[0].azimuth_angle;
        if(tmpAngleDiff == 0) {
            tmpAngleDiff = packet->groups[2].azimuth_angle - packet->groups[0].azimuth_angle;
            echo_flag = 2;
        }
        if (tmpAngleDiff < -35000){
            tmpAngleDiff += 36000;
        }
        if(DEVICE_ID==0x4F||DEVICE_ID==0x55||DEVICE_ID==0x56||DEVICE_ID==0x57||DEVICE_ID==0x58)
        {
            angular_z = tmpAngleDiff*UINTCONVERT/(32*3.125);
        }
        else if(DEVICE_ID == 0x5B || DEVICE_ID == 0x5C)
        {
            angular_z = tmpAngleDiff*UINTCONVERT/(32*3.33);
        }
        else
        {
            angular_z = tmpAngleDiff*UINTCONVERT/(32*1.5625);
        }
        for(int j=0; j< POINT_NUM_USER*2; j+=2)//16
        {
            switch (DEVICE_ID) {
            case RFANS_PRODUCT_MODEL_V6A_X16M_0X4F:
            case RFANS_PRODUCT_MODEL_V6A_E1_0X55:
            case RFANS_PRODUCT_MODEL_V6A_E2_0X56:
                timeOffset =3.125;
                if(j<15)
                {
                    if(echo_flag == 2){
                        tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j];//us
                        tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j+1];
                    }
                    else{
                        tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j];//us
                        tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j+1];
                    }
                    tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6A_X16M_0X4F_0x55_0x56[j];//deg
                    tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6A_X16M_0X4F_0x55_0x56[j+1];
                }
                else{
                    if (echo_flag == 2 ) {
                        tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-16]+50;
                        tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-15]+50;
                    }
                    else{
                        tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-16]+50;
                        tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-15]+50;
                    }
                    tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-16]+50);
                    tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-15]+50);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6BC_16G_0X57:
            case RFANS_PRODUCT_MODEL_V6BC_16M_0X58:
                timeOffset =3.125;
                if(j<15)
                {
                    if(echo_flag == 2) {
                        tmpXyz1.timeflag = packet->gps_timestamp + timeOffset*32*(i/2)+DELTA_T_V6BC_16GM_0x57_0x58[j];
                        tmpXyz2.timeflag = packet->gps_timestamp + timeOffset*32*(i/2)+DELTA_T_V6BC_16GM_0x57_0x58[j+1];
                    }
                    else {
                        tmpXyz1.timeflag = packet->gps_timestamp + timeOffset*32*i+DELTA_T_V6BC_16GM_0x57_0x58[j];
                        tmpXyz2.timeflag = packet->gps_timestamp + timeOffset*32*i+DELTA_T_V6BC_16GM_0x57_0x58[j+1];
                    }
                    tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6BC_16GM_0x57_0x58[j]);
                    tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6BC_16GM_0x57_0x58[j+1]);
                }
                else{
                    if(echo_flag == 2) {
                        tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6BC_16GM_0x57_0x58[j-16]+50;
                        tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6BC_16GM_0x57_0x58[j-15]+50;
                    }
                    else {
                        tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6BC_16GM_0x57_0x58[j-16]+50;
                        tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6BC_16GM_0x57_0x58[j-15]+50;
                    }
                    tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6BC_16GM_0x57_0x58[j-16]+50);
                    tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6BC_16GM_0x57_0x58[j-15]+50);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6K_16M_0X5B:
                timeOffset = 3.33;
                if(j < 15)
                {
                    if(echo_flag == 2){
                        tmpXyz1.timeflag = packet->gps_timestamp + timeOffset*32*(i/2)+DELTA_T_V6K_16M_0x5B[j];
                        tmpXyz2.timeflag = packet->gps_timestamp + timeOffset*32*(i/2)+DELTA_T_V6K_16M_0x5B[j+1];
                    }
                    else{
                        tmpXyz1.timeflag = packet->gps_timestamp + timeOffset*32*i+DELTA_T_V6K_16M_0x5B[j];
                        tmpXyz2.timeflag = packet->gps_timestamp + timeOffset*32*i+DELTA_T_V6K_16M_0x5B[j+1];
                    }
                    tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6K_16M_0x5B[j]);
                    tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6K_16M_0x5B[j+1]);

                }
                else{
                    if(echo_flag == 2) {
                        tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6K_16M_0x5B[j-16]+53.28;
                        tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6K_16M_0x5B[j-15]+53.28;
                    }
                    else {
                        tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6K_16M_0x5B[j-16]+53.28;
                        tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6K_16M_0x5B[j-15]+53.28;
                    }
                    tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6K_16M_0x5B[j-16]+53.28);
                    tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6K_16M_0x5B[j-15]+53.28);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6K_16G_0x5C:
                timeOffset = 3.33;
                if(j <15){
                    if(echo_flag == 2) {
                        tmpXyz1.timeflag = packet->gps_timestamp + timeOffset*32*(i/2)+DELTA_T_RFANS_V6K_16G_0x5C[j];
                        tmpXyz2.timeflag = packet->gps_timestamp + timeOffset*32*(i/2)+DELTA_T_RFANS_V6K_16G_0x5C[j+1];
                    }
                    else{
                        tmpXyz1.timeflag = packet->gps_timestamp + timeOffset*32*i+DELTA_T_RFANS_V6K_16G_0x5C[j];
                        tmpXyz2.timeflag = packet->gps_timestamp + timeOffset*32*i+DELTA_T_RFANS_V6K_16G_0x5C[j+1];
                    }
                    tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_RFANS_V6K_16G_0x5C[j]);
                    tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_RFANS_V6K_16G_0x5C[j+1]);
                }
                else{
                    if (echo_flag == 2){
                        tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_RFANS_V6K_16G_0x5C[j-16]+53.28;
                        tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_RFANS_V6K_16G_0x5C[j-15]+53.28;
                    }
                    else{
                        tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_RFANS_V6K_16G_0x5C[j-16]+53.28;
                        tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_RFANS_V6K_16G_0x5C[j-15]+53.28;
                    }
                    tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_RFANS_V6K_16G_0x5C[j-16]+53.28);
                    tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_RFANS_V6K_16G_0x5C[j-15]+53.28);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6G_X32_0X33:
            case RFANS_PRODUCT_MODEL_V6B_X32_0X50:
            case RFANS_PRODUCT_MODEL_V6K_32M_0X5A:
                timeOffset = 1.5625;
                if(echo_flag == 2) {
                    tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6G_V6BC_X32_0x33_0x50[j];
                    tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6G_V6BC_X32_0x33_0x50[j+1];
                }
                else {
                    tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6G_V6BC_X32_0x33_0x50[j];
                    tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6G_V6BC_X32_0x33_0x50[j+1];
                }
                tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6G_V6BC_X32_0x33_0x50[j];
                tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6G_V6BC_X32_0x33_0x50[j+1];
                break;
            case RFANS_PRODUCT_MODEL_V6_X32_0X40:
            case RFANS_PRODUCT_MODEL_V6_X16A_0X41:
            case RFANS_PRODUCT_MODEL_V6_X16B_0X42:
            case RFANS_PRODUCT_MODEL_V6_X16Even_0X43:
            case RFANS_PRODUCT_MODEL_V6_X16Odd_0X44:
            case RFANS_PRODUCT_MODEL_V6A_X32_0X4A:
            case RFANS_PRODUCT_MODEL_V6A_X16A_0X4B:
            case RFANS_PRODUCT_MODEL_V6A_X16B_0X4C:
            case RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D:
            case RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E:
                timeOffset = 1.5625;
                if (echo_flag == 2) {
                    tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6_X32_0x40[j];
                    tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6_X32_0x40[j+1];
                }
                else{
                    tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6_X32_0x40[j];
                    tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6_X32_0x40[j+1];
                }

                tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6_X32_0x40[j];
                tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6_X32_0x40[j+1];
                break;
            case RFANS_PRODUCT_MODEL_CFANS_X32_0X80:
                timeOffset = 1.5625;
                tmpXyz1.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_CFANS_0x80[j];
                tmpXyz2.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_CFANS_0x80[j+1];
                tmpAngle1 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_CFANS_0x80[j];
                tmpAngle2 = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_CFANS_0x80[j+1];
                break;

            default:
                break;
            }
            tmpRange1 = mtBlock->points[j/2].range1*CONVERT_4mm_2m;
            tmpRange2 = mtBlock->points[j/2].range2*CONVERT_4mm_2m;
            tmpXyz1.intent = (mtBlock->points[j/2].intents[0] << 4)
                    | ((mtBlock->points[j/2].intents[1] & 0xF0) >> 4);
            tmpXyz2.intent = ((mtBlock->points[j/2].intents[1] & 0x0F) << 8)
                    | mtBlock->points[j/2].intents[2];
            tmpXyz1.laserid = j;
            tmpXyz2.laserid = j+1;
            calcXyz(DEVICE_ID,tmpRange1,tmpAngle1,tmpXyz1);
            calcXyz(DEVICE_ID,tmpRange2,tmpAngle2,tmpXyz2);
            if(g_xyzFile) {
                fprintf(g_xyzFile,"%02d, %f, %f, %f\r\n",
                        tmpXyz1.laserid,tmpXyz1.x,tmpXyz1.y,tmpXyz1.z);
                fprintf(g_xyzFile,"%02d, %f, %f, %f\r\n",
                         tmpXyz2.laserid,tmpXyz2.x,tmpXyz2.y,tmpXyz2.z);
                fflush(g_xyzFile);
            }
            if(tmpRange1 > max_range || tmpRange1 < min_range|| tmpXyz1.hangle>max_angle || tmpXyz1.hangle<min_angle)
            {
                tmpXyz1.x =NAN;
                tmpXyz1.y = NAN;
                tmpXyz1.z = NAN;
            }
            if(tmpRange2 > max_range || tmpRange2 < min_range|| tmpXyz2.hangle>max_angle || tmpXyz2.hangle<min_angle)
            {
                tmpXyz2.x =NAN;
                tmpXyz2.y = NAN;
                tmpXyz2.z = NAN;
            }
            if(checkOneRound(tmpAngle1,tmpXyz1.laserid,outCloud)) rtn =1;
            s_lineData[s_lineCount] = tmpXyz1;
            ++s_lineCount;
            if(checkOneRound(tmpAngle2,tmpXyz2.laserid,outCloud)) rtn =1;
            s_lineData[s_lineCount] = tmpXyz2;
            ++s_lineCount;
            if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1;

        }
    }
    return rtn;
}

//This function parses and processes LIDAR data and finally generates point cloud data
inline int processPacketUserSimple(PACKET_USER_SIMPLE_S* packet, sensor_msgs::msg::PointCloud2 &outCloud)
{
    int rtn =0;
    RFANS_XYZ_S tmpXyz;
    const float CONVERT_4mm_2m =0.004f;
    float tmpAngle, tmpRange;
    float timeOffset = 1.5625;
    float angular_z;
    //    unsigned char DEVICE_ID = ((packet->factory)>>8)&0xff;
    unsigned char DEVICE_ID = packet->gmReservedA;
    uint8_t echo_flag = 1;
    gpsTime = packet->gps_timestamp;
    
    for(int i =0; i < GROUP_NUM_USER_SIMPLE; i++)
    {
        GROUP_USER_SIMPLE_S* mtBlock = &packet->groups[i];
        
        int tmpAngleDiff = packet->groups[1].azimuth_angle - packet->groups[0].azimuth_angle;
        if (tmpAngleDiff == 0){
            tmpAngleDiff = packet->groups[2].azimuth_angle - packet->groups[0].azimuth_angle;
            echo_flag = 2;
            // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dededebug == 0");
        }
        if (tmpAngleDiff < -35000)
        {
            tmpAngleDiff += 36000;
            // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dededebug == +36000");
        }
        if(DEVICE_ID==0x4F||DEVICE_ID==0x55||DEVICE_ID==0x56||DEVICE_ID==0x57||DEVICE_ID==0x58){
            angular_z = tmpAngleDiff*UINTCONVERT/(32*3.125);
            // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dededebug == 0x4f");
        }
        else if(DEVICE_ID==0x5C ||DEVICE_ID==0x5B){
            angular_z= tmpAngleDiff*UINTCONVERT/(32*3.33);//300K
            // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dededebug == 0x5C or 0x5B");
            //this else if true
        }
        else{
            angular_z = tmpAngleDiff*UINTCONVERT/(32*1.5625);
            // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dededebug == else");
        }
        
        for(int j=0; j< POINT_NUM_USER_SIMPLE; j++)
        {
            switch (DEVICE_ID) {
            case RFANS_PRODUCT_MODEL_V6A_X16M_0X4F:
            case RFANS_PRODUCT_MODEL_V6A_E1_0X55:
            case RFANS_PRODUCT_MODEL_V6A_E2_0X56:
                timeOffset =3.125;
                if(j<=15)
                {
                    if (echo_flag == 2){
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j];//us
                    }
                    else{
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j];//us
                    }
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6A_X16M_0X4F_0x55_0x56[j];//deg
                }
                else{
                    if(echo_flag == 2){
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-16]+50;
                    }
                    else{
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-16]+50;
                    }
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6A_X16M_0X4F_0x55_0x56[j-16]+50);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6BC_16G_0X57:
            case RFANS_PRODUCT_MODEL_V6BC_16M_0X58:
                timeOffset =3.125;
                if(j<=15)
                {
                    if (echo_flag == 2){
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6BC_16GM_0x57_0x58[j];
                    }
                    else{
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6BC_16GM_0x57_0x58[j];
                    }
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6BC_16GM_0x57_0x58[j];
                }
                else{
                    if(echo_flag == 2){
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6BC_16GM_0x57_0x58[j-16]+50;
                    }
                    else {
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6BC_16GM_0x57_0x58[j-16]+50;
                    }
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6BC_16GM_0x57_0x58[j-16]+50);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6K_16G_0x5C:
                timeOffset =3.33;
                
                //tmpAngle = (mtBlock->azimuth_angle+j*s_angleStep)*UINTCONVERT;
                if(j<=15)
                {
                    if(echo_flag == 2){
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_RFANS_V6K_16G_0x5C[j];
                    }else{
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_RFANS_V6K_16G_0x5C[j];
                    }
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_RFANS_V6K_16G_0x5C[j];
                    // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dededebug == 0x5C j<=15");
                }
                else{
                    if(echo_flag == 2){
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_RFANS_V6K_16G_0x5C[j-16]+53.28;
                    }
                    else{
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_RFANS_V6K_16G_0x5C[j-16]+53.28;
                    }
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_RFANS_V6K_16G_0x5C[j-16]+53.28);
                    // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dededebug == 0x5C j<=16");
                }
                break;
            case RFANS_PRODUCT_MODEL_V6K_16M_0X5B:
                // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dededebug == 0x5B");
                timeOffset =3.33;
                if(j<=15)
                {
                    if(echo_flag == 2){
                       tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6K_16M_0x5B[j];
                    }
                    else{
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6K_16M_0x5B[j];
                    }
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6K_16M_0x5B[j];
                }
                else{
                    if(echo_flag == 2){
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6K_16M_0x5B[j-16]+53.28;
                    }
                    else{
                        tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6K_16M_0x5B[j-16]+53.28;
                    }
                    tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*(DELTA_T_V6K_16M_0x5B[j-16]+53.28);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6G_X32_0X33:
            case RFANS_PRODUCT_MODEL_V6B_X32_0X50:
            case RFANS_PRODUCT_MODEL_V6K_32M_0X5A:
                timeOffset = 1.5625;
                if(echo_flag == 2){
                    tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6G_V6BC_X32_0x33_0x50[j];
                }
                else{
                    tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6G_V6BC_X32_0x33_0x50[j];
                }
                tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6G_V6BC_X32_0x33_0x50[j];
                break;
            case RFANS_PRODUCT_MODEL_V6_X32_0X40:
            case RFANS_PRODUCT_MODEL_V6_X16A_0X41:
            case RFANS_PRODUCT_MODEL_V6_X16B_0X42:
            case RFANS_PRODUCT_MODEL_V6_X16Even_0X43:
            case RFANS_PRODUCT_MODEL_V6_X16Odd_0X44:
            case RFANS_PRODUCT_MODEL_V6A_X32_0X4A:
            case RFANS_PRODUCT_MODEL_V6A_X16A_0X4B:
            case RFANS_PRODUCT_MODEL_V6A_X16B_0X4C:
            case RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D:
            case RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E:
                timeOffset = 1.5625;
                if(echo_flag == 2){
                    tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*(i/2)+DELTA_T_V6_X32_0x40[j];
                }
                else{
                    tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_V6_X32_0x40[j];
                }
                tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_V6_X32_0x40[j];
                break;
            case RFANS_PRODUCT_MODEL_CFANS_X32_0X80:
                timeOffset = 1.5625;
                tmpXyz.timeflag = packet->gps_timestamp+timeOffset*32*i+DELTA_T_CFANS_0x80[j];
                tmpAngle = mtBlock->azimuth_angle*UINTCONVERT+angular_z*DELTA_T_CFANS_0x80[j];
                break;

            default:
                break;
            }

            tmpXyz.intent = mtBlock->points[j].intensity;
            tmpXyz.laserid = j;
            tmpRange = mtBlock->points[j].range*CONVERT_4mm_2m;
            tmpXyz.mirrorid =0;
            calcXyz(DEVICE_ID,tmpRange,tmpAngle,tmpXyz);
            if(g_xyzFile) {
                fprintf(g_xyzFile,"%02d, %f, %f, %f\r\n",
                        tmpXyz.laserid,tmpXyz.x,tmpXyz.y,tmpXyz.z);
                fflush(g_xyzFile);
            }
            if(tmpRange > max_range || tmpRange < min_range || tmpXyz.hangle>max_angle || tmpXyz.hangle<min_angle)
            {
                tmpXyz.x =NAN;
                tmpXyz.y = NAN;
                tmpXyz.z = NAN;
            }
            if(use_laserSelection_){
                if(tmpXyz.laserid != ringID)
                {
                    tmpXyz.x =NAN;
                    tmpXyz.y = NAN;
                    tmpXyz.z = NAN;
                }
            }
            
             if (echo_flag ==2) {
                if(i%2 ==0){
                    if(checkOneRound(tmpAngle,tmpXyz.laserid,outCloud)) rtn =1;
                }
            }
            else {
                // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dededebug == else中:%d",j);
                if(checkOneRound(tmpAngle,tmpXyz.laserid,outCloud)) rtn =1;
                
            }
            
            // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dededebug == s_line:%d",  tmpXyz.laserid);
            s_lineData[s_lineCount] = tmpXyz;
            // s_lineData.push_back(tmpXyz);
            ++s_lineCount;
            if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1;
            
       
        }
    }

    return rtn;

}


inline int processFrameV6G(RFans_UDP32FRAMEV6G_S *mtFrame, sensor_msgs::msg::PointCloud2 &outCloud)
{
    int rtn = 0;
    bool tmp_isFull = false;
    float s_angleStep = 0;
    unsigned char mtSync;
    RFANS_XYZ_S tmpXyz ;
    gpsTime = mtFrame->gpsTimestamp;
    const float CONVERT_4mm_2_m =0.004f;

    unsigned int tmpDiftime = 0;
    float tmpAngle,tmpRange;
    float timeOffset = 1.5625; //us   640KHz
    float angular_z;
    unsigned char index=0;
    uint8_t echo_flag = 1;
    for (int j = 0; j < UDP32FRAMEV6G_COUNT; j++)//12Group
    {
        RFans_DataBlock_S *mtBlock = &mtFrame->dataBlock[j];
        mtSync = mtBlock->flag;


        int tmpAngleDif = mtFrame->dataBlock[1].azimuthAngle-mtFrame->dataBlock[0].azimuthAngle;
        if(tmpAngleDif == 0) {
            tmpAngleDif = mtFrame->dataBlock[2].azimuthAngle - mtFrame->dataBlock[0].azimuthAngle;
            echo_flag = 2;
        }
        if (tmpAngleDif< -35000) {
            tmpAngleDif += 36000;
        }
        if(mtFrame->gmReservedA==0x4F||mtFrame->gmReservedA==0x55||mtFrame->gmReservedA==0x56||mtFrame->gmReservedA==0x57||mtFrame->gmReservedA==0x58)
        {
            angular_z= tmpAngleDif*UINTCONVERT/(32*3.125);//deg/us
        }
        else if(mtFrame->gmReservedA==0x5C || mtFrame->gmReservedA==0x5B)
        {
            angular_z= tmpAngleDif*UINTCONVERT/(32*3.33);
        }
        // add the support of c-fans32
        else
        {
            angular_z= tmpAngleDif*UINTCONVERT/(32*1.5625);
        }
        for (int i = 0; i < 32; i++) {
            if (i % 2 == 0)            {
                index = (mtFrame->dataBlock[j].flag & 0xF000)>>12;
            }
            else{
                index = (mtFrame->dataBlock[j].flag & 0x0F00)>>8;
            }
            unsigned char mirrorID = (mtFrame->dataBlock[j].flag &0x000F)>>2;
            unsigned char device_Type = mtFrame->gmReservedA;
            switch (device_Type) {
            case RFANS_PRODUCT_MODEL_V6A_X16M_0X4F:
            case RFANS_PRODUCT_MODEL_V6A_E1_0X55:
            case RFANS_PRODUCT_MODEL_V6A_E2_0X56:
                timeOffset =3.125;
                if(i<=15)
                {

                    tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_V6A_X16M_0X4F_0x55_0x56[i];
                    tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*DELTA_T_V6A_X16M_0X4F_0x55_0x56[i];//deg
                }
                else{
                    tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_V6A_X16M_0X4F_0x55_0x56[i-16]+50;
                    tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*(DELTA_T_V6A_X16M_0X4F_0x55_0x56[i-16]+50);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6BC_16G_0X57:
            case RFANS_PRODUCT_MODEL_V6BC_16M_0X58:
                timeOffset =3.125;
                if(i<=15)
                {

                    tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_V6BC_16GM_0x57_0x58[i];
                    tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*DELTA_T_V6BC_16GM_0x57_0x58[i];
                }
                else{

                    tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_V6BC_16GM_0x57_0x58[i-16]+50;
                    tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*(DELTA_T_V6BC_16GM_0x57_0x58[i-16]+50);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6K_16G_0x5C:
                timeOffset =3.33;
                if(i<=15)
                {

                    tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_RFANS_V6K_16G_0x5C[i];
                    tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*DELTA_T_RFANS_V6K_16G_0x5C[i];
                }
                else{
                    tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_RFANS_V6K_16G_0x5C[i-16]+53.28;
                    tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*(DELTA_T_RFANS_V6K_16G_0x5C[i-16]+53.28);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6K_16M_0X5B:
                timeOffset =3.33;
                if(i<=15)
                {
                    tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_V6K_16M_0x5B[i];
                    tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*DELTA_T_V6K_16M_0x5B[i];
                }
                else{
                    tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_V6K_16M_0x5B[i-16]+53.28;
                    tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*(DELTA_T_V6K_16M_0x5B[i-16]+53.28);
                }
                break;
            case RFANS_PRODUCT_MODEL_V6G_X32_0X33:
            case RFANS_PRODUCT_MODEL_V6B_X32_0X50:
            case RFANS_PRODUCT_MODEL_V6K_32M_0X5A:
                timeOffset = 1.5625;
                tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_V6G_V6BC_X32_0x33_0x50[i];
                tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*DELTA_T_V6G_V6BC_X32_0x33_0x50[i];
                break;
            case RFANS_PRODUCT_MODEL_V6_X32_0X40:
            case RFANS_PRODUCT_MODEL_V6_X16A_0X41:
            case RFANS_PRODUCT_MODEL_V6_X16B_0X42:
            case RFANS_PRODUCT_MODEL_V6_X16Even_0X43:
            case RFANS_PRODUCT_MODEL_V6_X16Odd_0X44:
            case RFANS_PRODUCT_MODEL_V6A_X32_0X4A:
            case RFANS_PRODUCT_MODEL_V6A_X16A_0X4B:
            case RFANS_PRODUCT_MODEL_V6A_X16B_0X4C:
            case RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D:
            case RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E:
                timeOffset = 1.5625;
                tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_V6_X32_0x40[i];
                tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*DELTA_T_V6_X32_0x40[i];
                break;
            case RFANS_PRODUCT_MODEL_CFANS_X32_0X80:
                timeOffset = 1.5625;
                tmpXyz.timeflag = mtFrame->gpsTimestamp+timeOffset*32*(j/echo_flag)+DELTA_T_CFANS_0x80[i];
                tmpAngle = mtBlock->azimuthAngle*UINTCONVERT+angular_z*DELTA_T_CFANS_0x80[i];
                break;
            // get c-fans32 timeOffset,to get each point's timeflag.
            default:
                break;
            }


            tmpXyz.intent = mtBlock->laserBlock[i].intensity;

            tmpRange = mtBlock->laserBlock[i].range *CONVERT_4mm_2_m;

            tmpXyz.laserid = i;

            if(mtFrame->gmReservedA==RFANS_PRODUCT_MODEL_CFANS_X32_0X80){
                              tmpXyz.mirrorid=index ;
                calcCFansCoor(tmpRange,tmpAngle,index,i,tmpXyz) ;
            }else if(mtFrame->gmReservedA == CFNAS_32_0x81)
            {
                int ringID = i%8;
                calcCFansXYZ_32(tmpRange,tmpAngle,mirrorID,ringID,tmpXyz);
            }
            else
            {
                calcXyz(mtFrame->gmReservedA, tmpRange, tmpAngle, tmpXyz);
            }


            if(tmpRange > max_range || tmpRange < min_range || tmpXyz.hangle>max_angle || tmpXyz.hangle<min_angle)
            {
                tmpXyz.x =NAN;
                tmpXyz.y = NAN;
                tmpXyz.z = NAN;
            }
            if(use_laserSelection_){
                if(tmpXyz.laserid != ringID)
                {
                    tmpXyz.x =NAN;
                    tmpXyz.y = NAN;
                    tmpXyz.z = NAN;
                }
            }

            if(echo_flag == 2) {
                if(j%2 ==0) {
                    if ( checkFrame_sum(mtFrame->gmReservedA,tmpAngle,tmpXyz.laserid ,outCloud) ) rtn =1 ;
                }
            }

            else {
                if ( checkFrame_sum(mtFrame->gmReservedA,tmpAngle,tmpXyz.laserid ,outCloud) ) rtn =1 ;
            }

            //s_lineData[s_lineCount] = tmpXyz;
            s_rowData[i].push_back(tmpXyz);//按激光器通道保存对应点
            ++s_lineCount;
            if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1;
        }

    }
    //m_lastBloc
    return rtn;
}

const unsigned char SYNC_RELEASE_BLOCKV32_CFANS128_0_15 = 0x9D;
const unsigned char SYNC_RELEASE_BLOCKV32_CFANS128_16_31 = 0x9E;

inline int processFrameV5( RFans_UDPFRAMEV5_S *mtFrame, sensor_msgs::msg::PointCloud2 &outCloud)
{
    int rtn = 0;
    RFANS_XYZ_S tmpXyz ;
    float angleDif = 0 ,T0_STEP_VALUE = 0;
    float tmpAngle,tmpRange;
    for (int i = 0; i < 10; i++)
    {
        SCDRFANS_BLOCK_S *mtBlock = &mtFrame->blockdata[i];
        unsigned char tmpCheck = checkSum(((unsigned char*)mtBlock) + 2, sizeof(SCDRFANS_BLOCK_S)-2);
        if (tmpCheck != mtBlock->chksum) {  //check sum
            continue;
        }

        for(int j = 0 ; j <RFANS_LASER_COUNT;j++ ) {
            tmpXyz.laserid = j;

            switch (mtBlock->dataID ) {
            case ID_RFANSBLOCKV32_16_31_SYNC:
            case ID_RFANSBLOCKV6G_16_31_SYNC:
            case ID_RFANSBLOCKV32_0_15_SYNC:
            case ID_RFANSBLOCKV6G_0_15_SYNC:
            case SYNC_RELEASE_BLOCKV32_CFANS128_0_15:
            case SYNC_RELEASE_BLOCKV32_CFANS128_16_31:
                T0_STEP_VALUE = 15.625;
                if(mtBlock->dataID==ID_RFANSBLOCKV32_16_31_SYNC || mtBlock->dataID==ID_RFANSBLOCKV6G_16_31_SYNC || mtBlock->dataID== SYNC_RELEASE_BLOCKV32_CFANS128_16_31)
                    tmpXyz.laserid+=RFANS_LASER_COUNT;
                break;
            case ID_RFANSBLOCKV2_SYNC:
                T0_STEP_VALUE = 31.25;
                break;
            }

            tmpXyz.timeflag = mtBlock->t0stampH + TIME_FLAG_SCALE*(mtBlock->t0stampL + T0_STEP_VALUE*j);
            tmpAngle = mtBlock->laserData[j].angle *UINTCONVERT;
            tmpXyz.hangle = tmpAngle;
            tmpRange = mtBlock->laserData[j].rangeOne*UINTCONVERT;
            tmpXyz.intent  = mtBlock->laserData[j].intentTwo;
            if(mtBlock->dataID==SYNC_RELEASE_BLOCKV32_CFANS128_0_15 || mtBlock->dataID==SYNC_RELEASE_BLOCKV32_CFANS128_16_31){
                int index=calCFansMirrorIndex(tmpAngle,tmpXyz.laserid,tmpRange) ;
                calcCFansCoor(tmpRange,tmpAngle,index,tmpXyz.laserid,tmpXyz) ;
            }else{
                calcXyz(mtBlock->dataID,tmpRange, tmpAngle, tmpXyz);
            }


            if (tmpXyz.x == 0.0 && tmpXyz.y == 0.0 && tmpXyz.z == 0.0) {
                tmpXyz.x = tmpXyz.y = tmpXyz.z = NAN;
            }

            if ( checkFrame_sum(mtBlock->dataID,tmpAngle,tmpXyz.laserid ,outCloud) ) rtn =1 ;
            //s_lineData[s_lineCount] = tmpXyz;
            s_rowData[i].push_back(tmpXyz);//按激光器通道保存对应点
            ++s_lineCount;
            if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1;



        }
    }
    return rtn;
}

SSBufferDec::SSBufferDec()
{
    reset();
}

SSBufferDec::~SSBufferDec()
{

}



int SSBufferDec::moveWriteIndex(int setpIndex)
{
    m_decBuf.bufSize += setpIndex;
    m_udpSize = setpIndex;
    m_udpCount++;
    m_decBuf.wrHead = (m_decBuf.wrHead+setpIndex)%DECODE_BUFFER_SIZE;
    return m_decBuf.bufSize ;
}

int SSBufferDec::moveReadIndex(int setpIndex)
{
    m_decBuf.bufSize -= setpIndex;
    m_decBuf.rdTail = (m_decBuf.rdTail+setpIndex)%DECODE_BUFFER_SIZE;
    return m_decBuf.bufSize ;
}

unsigned char *SSBufferDec::getWriteIndex()
{
    return m_decBuf.buffer+m_decBuf.wrHead ;
}

unsigned char *SSBufferDec::getReadIndex()
{
    return m_decBuf.buffer+m_decBuf.rdTail ;
}


int SSBufferDec::writeBuffer(unsigned char *data, int size)
{
    if(m_decBuf.bufSize+size >= DECODE_BUFFER_SIZE) return 0 ;

    memcpy(m_decBuf.buffer+m_decBuf.wrHead,data,size);
    m_decBuf.bufSize += size;
    m_decBuf.wrHead += size ;

    //  ROS_INFO_STREAM( "writeBuffer"
    //                  << " bufSize " << m_decBuf.bufSize
    //                  << " wrHead "<< m_decBuf.wrHead
    //                  << " rdTail " <<m_decBuf.rdTail);
    return size ;
}

int SSBufferDec::readPacket(rfans_driver_msgs::msg::RfansPacket &pkt)
{
    int rtn = 0;
    if(m_decBuf.bufSize > 0 ) {
        pkt.data.resize(m_decBuf.bufSize);
        memcpy(&pkt.data[0], m_decBuf.buffer,  m_decBuf.bufSize);
        pkt.udpcount = m_udpCount;
        pkt.udpsize = m_udpSize;
        reset();
        rtn = 1;
    }
    return rtn ;
}

int SSBufferDec::size()
{
    return m_decBuf.bufSize;
}

int SSBufferDec::freeSize()
{
    return DECODE_BUFFER_SIZE-m_decBuf.bufSize;
}


static char s_tmpBuffer[DECODE_BUFFER_SIZE];

void SSBufferDec::bufferReverse()
{
    if (m_decBuf.bufSize > 0) {
        memcpy(s_tmpBuffer, m_decBuf.buffer + m_decBuf.rdTail, m_decBuf.bufSize);
        memcpy(m_decBuf.buffer, s_tmpBuffer, m_decBuf.bufSize);
        m_decBuf.rdTail = 0;
        m_decBuf.wrHead = m_decBuf.bufSize;
    } else {
        m_decBuf.wrHead = m_decBuf.bufSize = m_decBuf.rdTail = 0;
    }
    return;
}

void SSBufferDec::reset()
{
    //memset(&m_decBuf,0,sizeof(m_decBuf)) ;
    m_decBuf.bufSize = m_decBuf.wrHead = m_decBuf.rdTail = 0 ;
    memset(&m_packet,0,sizeof(m_packet)) ;
    m_status = eReady;
    s_preAngle =0 ;
    m_packetSize = 0;
    m_blockCout = 0 ;
    m_udpCount = 0 ;
}

int SSBufferDec::Depacket( const rfans_driver_msgs::msg::RfansPacket::SharedPtr &inPack, sensor_msgs::msg::PointCloud2 &outCloud , 
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &rosOut, DEVICE_TYPE_E deviceType)
{
    int rtn =0, updateflag = 0;
    s_device_type = deviceType;

    RFans_UDP32FRAMEV6G_S *tmpFrameV6;
    RFans_UDPFRAMEV5_S * tmpFrameV5;
    PACKET_ORI_S* tmpFrameORI;
    PACKET_USER_S* tmpFrameUSER;
    PACKET_USER_SIMPLE_S* tmpFrameUSERSimple;
    unsigned short* flag;
    float firstPointAngle = 0.0;

    if(UDP_PACKET_SIZE_DATA_LEVEL_ORI == inPack->udpsize)
    {
        for(int i =0;  i<inPack->udpcount;i++)
        {
            if(sizeof(PACKET_ORI_S)!=1406)

            tmpFrameORI = (PACKET_ORI_S*)(&inPack->data[0]+i*inPack->udpsize);

                if(processPacketOri(tmpFrameORI,outCloud))
                {

                    rosOut->publish(outCloud);
                    SSBufferDec::ResetPointCloud2(outCloud);
                }

        }

    }
    // if(UDP_PACKET_SIZE_V6G == inPack->udpsize)
    else if(UDP_PACKET_SIZE_V6G == inPack->udpsize)
    {
        
        for(int i=0 ; i< inPack->udpcount; i++)
        {
            flag = (unsigned short*)(&inPack->data[0]+i*inPack->udpsize);
            uint8_t data_grade, calc_grade, mirror_flag, pack_grade;
            data_grade = (*flag >> 14) & 0x0003;//2bit
            calc_grade = (*flag>> 4) & 0x03FF;//10bit
            mirror_flag = (*flag >> 2) & 0x0003;//2bit
            pack_grade = *flag & 0x0003;//2bit

            if(data_level_==2)
            {
                tmpFrameUSER =(PACKET_USER_S*)(&inPack->data[0]+i*inPack->udpsize);
                if(processPacketUser(tmpFrameUSER,outCloud))
                {
                    rosOut->publish(outCloud);
                    SSBufferDec::ResetPointCloud2(outCloud);
                }
            }
            else if(data_level_ ==3)
            // if(data_level_ ==3)
            {
                tmpFrameUSERSimple =(PACKET_USER_SIMPLE_S*) (&inPack->data[0]+i*inPack->udpsize);
                if(processPacketUserSimple(tmpFrameUSERSimple,outCloud))
                {
                    // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"outcloud:%d",outCloud.width);
                    rosOut->publish(outCloud);
                    SSBufferDec::ResetPointCloud2(outCloud);
                }
                // RCLCPP_INFO(rclcpp::get_logger("calculation_node"),"dedede");
            }
            
            else
            {
                tmpFrameV6 = (RFans_UDP32FRAMEV6G_S*)(&inPack->data[0] + i*inPack->udpsize);
                firstPointAngle = tmpFrameV6->dataBlock[0].azimuthAngle*UINTCONVERT;
                static unsigned int lastGpsTimestamp = 0;
                bool recordPPSAngle = recordAsPPSAngel(lastGpsTimestamp, tmpFrameV6->gpsTimestamp);
                if (recordPPSAngle) {
                    // ROS_INFO_STREAM("The device IP is: "<<ip_address<<"\n"
                    //                 <<"                                The pps angle is: "<<firstPointAngle);
                    // RCLCPP_INFO_STREAM_ONCE(th"The device IP is: "<<ip_address<<"\n"
                    //                 <<"                                The pps angle is: "<<firstPointAngle);
                    //                ROS_INFO_STREAM("The pps angle is "<<firstPointAngle);
                }
                lastGpsTimestamp = tmpFrameV6->gpsTimestamp;
                if( processFrameV6G(tmpFrameV6,outCloud) ) {
                    rosOut->publish(outCloud);
                    SSBufferDec::ResetPointCloud2(outCloud);
                }
            }     
        }
    }
    return rtn ;
}

void SSBufferDec::InitPointcloud2(sensor_msgs::msg::PointCloud2 &initCloud, std::string &frame_id_str) {
    static const size_t DataSize = sizeof(rfans_driver_msgs::msg::RfansPacket().data) / sizeof(SCDRFANS_BLOCK_S ) * sizeof(RFANS_XYZ_S) *RFANS_LASER_COUNT;
    initCloud.data.clear();
    initCloud.data.resize( DataSize); //point data

    initCloud.is_bigendian = false ;//false;      //stream foramt
    initCloud.fields.resize(8);          //line format
    initCloud.is_dense = false;

    int tmpOffset = 0 ;
    for(int i=0; i < initCloud.fields.size() ;i++) {
        switch(i) { //value type
        case 0:
            initCloud.fields[i].name = "x" ;
            initCloud.fields[i].datatype = 7u;
            break;
        case 1:
            initCloud.fields[i].name = "y" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 2:
            initCloud.fields[i].name = "z" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 3:
            initCloud.fields[i].name = "intensity" ;
            initCloud.fields[i].datatype = 7u;//2u;
            tmpOffset += 4;
            break;
        case 4:
            initCloud.fields[i].name = "laserid" ;
            initCloud.fields[i].datatype = 5u;
            tmpOffset += 4;
            break;
        case 5:
            initCloud.fields[i].name = "timeflag" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 6:
            initCloud.fields[i].name = "hangle" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 7:
            initCloud.fields[i].name = "mirrorid" ;
            initCloud.fields[i].datatype = 2u;
            tmpOffset += 1;
            break;

        }
        initCloud.fields[i].offset = tmpOffset ;      //value offset
        initCloud.fields[i].count = 1 ;
    }
    initCloud.height = 1;
    initCloud.point_step = sizeof(RFANS_XYZ_S);
    initCloud.row_step = DataSize ;
    initCloud.width = 0 ;


    //node name
    // std::string node_name =  rclcpp::Node::get_name();

    // std::string frame_id_str = "/world";
    // std::string frame_id_path = node_name + "/frame_id";
    // std::string ip_str = std::string("rfans_driver/") + "device_ip";
    // std::string data_level_param = std::string("rfans_driver/")+"data_level";

    // get_parameter(frame_id_path,frame_id_str);
    // get_parameter(ip_str,ip_address);
    // get_parameter(data_level_param,data_level_);
    // get_parameter("model",device_type);


    initCloud.header.frame_id = frame_id_str;

    s_lastAngle = 0 ;
    s_lineData.resize(LINE_POINT_COUNT);
    s_lineCount = 0 ;

    s_hangles.clear();
    s_vangles.clear();
    for (int i = 0; i < 32; ++i) {
        s_rowData[i].clear();
    }

}

void SSBufferDec::ResetPointCloud2(sensor_msgs::msg::PointCloud2 &initCloud) {
    initCloud.width = 0;
}

void SSBufferDec::SetAngleDuration(float value)
{
    if(value <10  || value > 360)
        return ;
    s_angle_duration = value;
}

void SSBufferDec::setSaveXYZ(bool save) {

    if (true == save && !g_xyzFile) {
        time_t now; struct tm *timeNow;
        time(&now);
        timeNow = localtime(&now);
        int yy = timeNow->tm_year % 100;
        int mm = (timeNow->tm_mon + 1) % 12;
        int dd = timeNow->tm_mday;
        int hh = timeNow->tm_hour;
        int mn = timeNow->tm_min;
        int ss = timeNow->tm_sec;
        char filename[FILENAME_MAX] = { '\0' };
        sprintf(filename, "xyz-%02d%02d%02d-%02d%02d%02d.txt", yy, mm, dd, hh, mn, ss);

        g_xyzFile = fopen(filename, "w+");
        fprintf(g_xyzFile,"id, X, Y, Z, Intent, Timeflag \r\n");
        fflush(g_xyzFile);
    }
}

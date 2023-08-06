#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rfans_driver/ssFrameLib.hpp"
// #include "rfans_driver_msgs/msg/rfans_packet.hpp"

const int SIZE_RFANS_DATA = sizeof(RFANS_XYZ_S);

class CloudProcessor : public rclcpp::Node
{
public:
  CloudProcessor() : Node("cloud_process_node")
  {
    InitPointcloud2(msg_pub);

    sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rfans_driver/rfans_points", 10, std::bind(&CloudProcessor::cloudCallback, this, std::placeholders::_1));

    pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("surestar_points", 10);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    
    msg_pub->header = msg->header;
    msg_pub->width = msg->width;
    msg_pub->height = msg->height;

    msg_pub->point_step = 20;
    int data_size = msg_pub->width * msg_pub->point_step;
    msg_pub->data.resize(data_size);
    msg_pub->row_step = msg_pub->data.size();
    for (size_t i = 0; i < msg_pub->width; i++)
    {
      memcpy(&msg_pub->data[i * msg_pub->point_step], &msg->data[i * SIZE_RFANS_DATA], msg_pub->point_step);
    }
    pub->publish(*msg_pub);
  }

  void InitPointcloud2(sensor_msgs::msg::PointCloud2::SharedPtr initCloud)
  {
    static const size_t DataSize = sizeof(rfans_driver_msgs::msg::RfansPacket().data) / sizeof(SCDRFANS_BLOCK_S) *
                                   sizeof(RFANS_XYZ_S) * RFANS_LASER_COUNT;
    initCloud->data.clear();
    initCloud->data.resize(DataSize); // point data

    initCloud->is_bigendian = false; // false;      // stream format
    initCloud->fields.resize(5);     // line format
    initCloud->is_dense = false;

    int tmpOffset = 0;
    for (int i = 0; i < initCloud->fields.size(); i++)
    {
      switch (i)
      { // value type
        case 0:
          initCloud->fields[i].name = "x";
          initCloud->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
          break;
        case 1:
          initCloud->fields[i].name = "y";
          initCloud->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
          tmpOffset += 4;
          break;
        case 2:
          initCloud->fields[i].name = "z";
          initCloud->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
          tmpOffset += 4;
          break;
        case 3:
          initCloud->fields[i].name = "intensity";
          initCloud->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
          tmpOffset += 4;
          break;
        case 4:
          initCloud->fields[i].name = "ring";
          initCloud->fields[i].datatype = sensor_msgs::msg::PointField::INT32;
          tmpOffset += 4;
          break;
      }
      initCloud->fields[i].offset = tmpOffset; // value offset
      initCloud->fields[i].count = 1;
    }
    initCloud->height = 1;
    initCloud->point_step = 20;
    initCloud->row_step = DataSize;
    initCloud->width = 0;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
  sensor_msgs::msg::PointCloud2::SharedPtr msg_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudProcessor>());
  rclcpp::shutdown();
  return 0;
}

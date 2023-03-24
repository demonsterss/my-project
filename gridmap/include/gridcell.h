#ifndef GRIDCELL_H__
#define GRIDCELL_H__

#include <iostream>
#include <vector>
#include <string>
// ROS
#include "ros/ros.h"
#include <opencv2/opencv.hpp>

#include <chrono>
#include <set>
#include <cstring>
#include <ros/package.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <typeinfo> 
#include <fstream>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/GridCells.h>
//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#pragma comment(lib,"ws2_32.lib")

typedef pcl::PointXYZ PointT;

using namespace std;

#define GRID_SIZE 15000
#define DELAY 1000
#define GRID_WIDTH 200
#define GRID_HEIGHT 520

typedef struct Yaml_data
{
    std::string ip_addr_rcv;
    int port_rcv;
    std::string ip_addr_send;
    int port_send;
    int obj_size;

	int grid_size;
	int y_max;
	double z_max;
	double z_min;
	int x_max;
	int x_min;
	double car_y;
	double car_x_min;
	double car_x_max;
	// int grid_width;
	// int grid_height;
	int grid_n1;
	int grid_n1_param;
	int grid_n2;
	int grid_n2_param;
	int grid_n3;
	int grid_n3_param;
	int grid_n4;
	int grid_n4_param;
	int grid_n5;
	int grid_n5_param;
	int grid_n6;
	int grid_n6_param;
	int grid_n7;
	int grid_n7_param;
};

struct Point3f
{
double x;
double y;
double z;
};

typedef struct
{
  uint16_t m;
  uint16_t n;
} Gridcell;


typedef struct
{
  Gridcell gridcell[GRID_SIZE];
  uint16_t last_bag;
  uint16_t id;
} Send_data;

class Gridcell_map
{
private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_result_sub;

    ros::Publisher pub;
public:
    Gridcell_map(/* args */);
    ~Gridcell_map();
    void gridcellInit();
    void pubcell();
    int YamlInit();
    int SocketInit();

    // yaml
    Yaml_data yaml_data;

    int sock_fd;
    
    nav_msgs::GridCells cells;

    struct sockaddr_in clientAddr;
    int send_num;

    void GridcellCallback(const sensor_msgs::PointCloud2ConstPtr &result_msg);
	int grid_pub(Send_data &data, int &gridpacket_index, int last_bag_flag, int data_index);
	void grid_input(Send_data &data, int grid_packet_index_begin, int grid_packet_index_end, int grid_param, int &grid_packet_index);


public:

	int gridmap[GRID_WIDTH][GRID_HEIGHT] = { 0 };
	double gridmap_max[GRID_WIDTH][GRID_HEIGHT] = { 0 };
	double gridmap_min[GRID_WIDTH][GRID_HEIGHT] = { 0 };
	int send_num1;
	int send_num2;
	int send_num3;
	int send_num4;
	int send_num5;
	int send_num6;
	int send_num7;
	int len;
	std::vector<Point3f> GridMap;
	Send_data send_data1;
	Send_data send_data2;
	Send_data send_data3;
	Send_data send_data4;
	Send_data send_data5;
	Send_data send_data6;
	Send_data send_data7;
	Send_data send_data_new;

};



#endif // !GRIDCELL_H__

#include "ros/ros.h"
#include <iostream>
#include "gridcell.h"

int grid_packet_index1 = 0;
int grid_packet_index2 = 0;
int grid_packet_index3 = 0;
int grid_packet_index4 = 0;
int grid_packet_index5 = 0;
int grid_packet_index6 = 0;
int grid_packet_index7 = 0;

Gridcell_map::Gridcell_map(/* args */)
{
    lidar_result_sub = nh.subscribe("fusion_lidar_points", 1000, &Gridcell_map::GridcellCallback,this);
    pub = nh.advertise<nav_msgs::GridCells>("/gridCell",1);
}

Gridcell_map::~Gridcell_map()
{
    close(sock_fd);
}

void Gridcell_map::gridcellInit()
{
    cout << "~~ ================================================ ~~" << endl;
    cout << "~~ =========       starting  Gridmap      ========= ~~" << endl;
    cout << "~~ ================================================ ~~" << endl;
    int yaml_ret = YamlInit();
    int socket_ret = SocketInit();
    cells.header.frame_id = "base_link";
	  cells.cell_height = 0.1;
	  cells.cell_width = 0.1;
}


void Gridcell_map::GridcellCallback(const sensor_msgs::PointCloud2ConstPtr &result_msg)
{
	cout<< "~~ ========recived message================ ~~"<<endl;
	double begin = ros::Time::now().toSec();
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*result_msg, *temp_cloud);
	cout<<"rev pointcloud size = "<<temp_cloud->points.size()<<endl;
    if (temp_cloud->points.size() > 0)
    {
      for (uint32_t i = 0; i < temp_cloud->points.size(); i++)
      {
        PointT pt = temp_cloud->points[i];
        if(abs(pt.y)<=yaml_data.car_y && pt.x>yaml_data.car_x_min && pt.x<=yaml_data.car_x_max){
          continue;
        }
        if (abs(pt.y) <= yaml_data.y_max && pt.x > yaml_data.x_min && pt.x <= yaml_data.x_max)
        {
          if ((pt.z > 0 && pt.z > yaml_data.z_min && pt.z < yaml_data.z_max) || (pt.z <0 && abs(pt.z) >yaml_data.z_min && abs(pt.z) < yaml_data.z_max))
          {
            int ttt1 = floor(-pt.y * yaml_data.grid_size) + yaml_data.y_max * yaml_data.grid_size;
            int ttt2 = floor(pt.x * yaml_data.grid_size);
            gridmap[ttt1][ttt2] = gridmap[ttt1][ttt2] + 1;
            if(pt.z > gridmap_max[ttt1][ttt2])
            {
              gridmap_max[ttt1][ttt2] = pt.z;
            }
            if(pt.z < gridmap_min[ttt1][ttt2])
            {
              gridmap_min[ttt1][ttt2] = pt.z;
            }
          }
        }
      }
    }

	grid_input(send_data1, 0, yaml_data.grid_n1, yaml_data.grid_n1_param, grid_packet_index1);
	grid_input(send_data2, yaml_data.grid_n1, yaml_data.grid_n2, yaml_data.grid_n2_param, grid_packet_index2);
	grid_input(send_data3, yaml_data.grid_n2, yaml_data.grid_n3, yaml_data.grid_n3_param, grid_packet_index3);
	grid_input(send_data4, yaml_data.grid_n3, yaml_data.grid_n4, yaml_data.grid_n4_param, grid_packet_index4);
	grid_input(send_data5, yaml_data.grid_n4, yaml_data.grid_n5, yaml_data.grid_n5_param, grid_packet_index5);
	grid_input(send_data6, yaml_data.grid_n5, yaml_data.grid_n6, yaml_data.grid_n6_param, grid_packet_index6);
	grid_input(send_data7, yaml_data.grid_n6, yaml_data.grid_n7, yaml_data.grid_n7_param, grid_packet_index7);

	cout<<"sub gridmap1 size is "<<grid_packet_index1<<endl;
	cout<<"sub gridmap2 size is "<<grid_packet_index2<<endl;
	cout<<"sub gridmap3 size is "<<grid_packet_index3<<endl;
	cout<<"sub gridmap4 size is "<<grid_packet_index4<<endl;
	cout<<"sub gridmap5 size is "<<grid_packet_index5<<endl;
	cout<<"sub gridmap6 size is "<<grid_packet_index6<<endl;
	cout<<"sub gridmap7 size is "<<grid_packet_index7<<endl;

	send_num1 = grid_pub(send_data1, grid_packet_index1, 0, 1);
	send_num2 = grid_pub(send_data2, grid_packet_index2, 0, 2);
	send_num3 = grid_pub(send_data3, grid_packet_index3, 0, 3);
	send_num4 = grid_pub(send_data4, grid_packet_index4, 0, 4);
	send_num5 = grid_pub(send_data5, grid_packet_index5, 0, 5);
	send_num6 = grid_pub(send_data6, grid_packet_index6, 0, 6);
	send_num7 = grid_pub(send_data7, grid_packet_index7, 1, 7);

    for (int i = 0; i < GRID_WIDTH; i++)
    {
      for (int j = 0; j < GRID_HEIGHT; j++)
      {
        gridmap[i][j] = 0;
        gridmap_max[i][j] = 0;
        gridmap_min[i][j] = 0;
      }
    }

	double end = ros::Time::now().toSec();
	double time = (end-begin);
	cout<<"using time = "<<time<<" S"<<endl;   
}

void Gridcell_map::pubcell()
{
    cout<<"gridcells ros pub size: "<<cells.cells.size()<<endl;
    if(cells.cells.size()>0)
    {
       pub.publish(cells);
    }
   cells.cells.clear();
}

int Gridcell_map::grid_pub(Send_data &data, int &gridpacket_index, int last_bag_flag, int data_index)
{
	int send_num;
    if (GRID_SIZE > gridpacket_index)
    {
      for (int k = gridpacket_index; k < GRID_SIZE; k++)
      {
        data.gridcell[k].m = 10000;
        data.gridcell[k].n = 10000;
      }
    }
    data.last_bag = last_bag_flag;
    data.id = data_index;
    send_num = sendto(sock_fd, &data, sizeof(Send_data), 0, (struct sockaddr*)&clientAddr, len);
    cout << "-----send_packet index: " << data_index << "-----send_num: " << send_num <<endl;
    data = send_data_new;
    gridpacket_index = 0;
    usleep(DELAY);
	return send_num;
}

void Gridcell_map::grid_input(Send_data &data, int grid_packet_index_begin, int grid_packet_index_end, int grid_param, int &grid_packet_index)
{
	for (int i = 0; i < GRID_WIDTH; i++)
    {
		for (int j = grid_packet_index_begin; j < grid_packet_index_end; j++)
    	{
        	if (gridmap[i][j] > grid_param && (gridmap_max[i][j] - gridmap_min[i][j]) > 0.15)
        	{
          		data.gridcell[grid_packet_index].m = i;
          		data.gridcell[grid_packet_index].n = j + 300;
          		grid_packet_index++;
             
             geometry_msgs::Point obstacle;
             obstacle.y = (-0.1) *(i - 100);
             obstacle.x = (0.1) *(j);
						obstacle.z = 0;
						cells.cells.push_back(obstacle);
        	}
		}
	}
	//cout<<"sub gridmap1 size is "<<grid_packet_index<<endl;
}

int Gridcell_map::YamlInit()
{
	const char conf_file[100] = "/home/demon/catkin_ws/src/gridmap/include/config.yaml";
    cv::FileStorage fs;
    fs.open(conf_file, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        cout << "Open the config file failed" << endl;
        return -1;
    }
    else
    {
        cout << "Open the config file success" << endl;
    }

    yaml_data.ip_addr_send = (std::string)fs["ip_addr_send"];
    std::cout << "ip_addr_send: " << yaml_data.ip_addr_send.c_str() << std::endl;

    yaml_data.port_send = (int)fs["port_send"];
    std::cout << "port_send: " << yaml_data.port_send << std::endl;

	yaml_data.y_max = (int)fs["y_max"];
	yaml_data.z_max = (double)fs["z_max"];
	yaml_data.z_min = (double)fs["z_min"];
	yaml_data.x_max = (int)fs["x_max"];
	yaml_data.x_min = (int)fs["x_min"];

	std::cout << "y_max: " << yaml_data.y_max << std::endl;
	std::cout << "x_max: " << yaml_data.x_max << std::endl;

	yaml_data.grid_size = (int)fs["grid_size"];

	yaml_data.car_y = (double)fs["car_y"];
	yaml_data.car_x_max = (double)fs["car_x_max"];
	yaml_data.car_x_min = (double)fs["car_x_min"];

	yaml_data.grid_n1 = (int)fs["grid_n1"];
	yaml_data.grid_n1_param = (int)fs["grid_n1_param"];

	yaml_data.grid_n2 = (int)fs["grid_n2"];
	yaml_data.grid_n2_param = (int)fs["grid_n2_param"];

	yaml_data.grid_n3 = (int)fs["grid_n3"];
	yaml_data.grid_n3_param = (int)fs["grid_n3_param"];

	yaml_data.grid_n4 = (int)fs["grid_n4"];
	yaml_data.grid_n4_param = (int)fs["grid_n4_param"];

	yaml_data.grid_n5 = (int)fs["grid_n5"];
	yaml_data.grid_n5_param = (int)fs["grid_n5_param"];

	yaml_data.grid_n6 = (int)fs["grid_n6"];
	yaml_data.grid_n6_param = (int)fs["grid_n6_param"];

	yaml_data.grid_n7 = (int)fs["grid_n7"];
	yaml_data.grid_n7_param = (int)fs["grid_n7_param"];

	std::cout << "grid_n7: " << yaml_data.grid_n7 << std::endl;
	std::cout << "grid_n7_param: " << yaml_data.grid_n7_param << std::endl;

    return 0;
}

int Gridcell_map::SocketInit()
{
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }
    memset(&clientAddr, 0, sizeof(clientAddr));
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_addr.s_addr = inet_addr(yaml_data.ip_addr_send.c_str());
    clientAddr.sin_port = htons(yaml_data.port_send); //port
    len = sizeof(clientAddr);
    return 0;
}


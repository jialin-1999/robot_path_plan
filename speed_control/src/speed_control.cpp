#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <fcntl.h>
#include <sys/shm.h>

#define BUFFER_SIZE 16    // two double variables
using namespace std;
static ofstream log_lidar("/home/sucro/log_lidar.csv");
double speed = 0.0;    // revise
double angle = 0.0;    // revise
double speed_ori = 0.0;    // revise
double angle_ori = 0.0;    // revise
struct dog_msg
{
	double angle;
	double speed;
};

void command_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	speed = msg->linear.x;
	angle = msg->angular.z;
	//speed = 1.0;
	//angle = 2.0;
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "speed_control");
	ros::NodeHandle nh;
	ros::Subscriber sub_command = nh.subscribe("/cmd_vel", 1, &command_callback);

 	// create a buffer to store the send message
 	// char sendbuf[BUFFER_SIZE];
 	// memset(sendbuf, 0, sizeof(sendbuf));

	// create a client socket cocket_client; params: ipv4\SOCK_STREAM\IPPROTO_TCP
	int socket_client = socket(AF_INET,SOCK_STREAM, 0);

	if( socket_client == -1)
	{
		printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
		exit(0);
	}
	// set socket address
	struct sockaddr_in  servaddr;
	std::string ip = "192.168.1.20";
	int port = 1986;
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(port);
	servaddr.sin_addr.s_addr = inet_addr(ip.c_str());

	if( inet_pton(AF_INET, ip.c_str(), &servaddr.sin_addr) <= 0)
	{
		printf("inet_pton error for %s\n", ip.c_str());
		exit(0);
	}
    // std::string ip = "127.0.0.1";
    // int port = 7000;

	// connect server; succuss return 0, defeat return -1
	if(connect(socket_client, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
	{
		printf("connect sever error: %s(errno: %d)\n", strerror(errno),errno);;
		exit(1);
	}

	printf("Connect OK! \n");

	ros::Rate loop_rate(10);
	while(ros::ok()){
		ros::spinOnce();

		dog_msg send_msg;
        if (fabs(angle_ori-angle) > 0.03) 
		{
             if (angle_ori-angle>0)
             {
                send_msg.angle=angle_ori-0.03;
				angle = angle_ori-0.03;
			 }
			 else
			 {
				 send_msg.angle=angle_ori+0.03;
				 angle = angle_ori+0.03;
			 }
			 
			 
		}
		else
		{
			send_msg.angle=angle;
		}
		
		
		send_msg.speed=speed;
		log_lidar<<angle<<",";
		log_lidar<<endl;
		if(send(socket_client, &send_msg, 16, 0) < 0)
		{
			printf("send msg error: %s (errno %d)\n", strerror(errno), errno);
			continue;
		}
		else
		{
			printf("send: speed = %lf   angle = %lf \n", speed, angle);
		}
		// memset(sendbuf, 0, sizeof(sendbuf));
		loop_rate.sleep();
		angle_ori = angle;
		speed_ori = speed;
	}

	close(socket_client);
	return 0;
}

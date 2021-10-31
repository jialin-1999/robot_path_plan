// #include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <geometry_msgs/Pose.h>

// #include <sys/types.h>
// #include <sys/socket.h>
// #include <stdio.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
// #include <unistd.h>
// #include <string.h>
// #include <stdlib.h>
// #include <stdio.h>
// #include <algorithm>
// #include <fstream>
// #include <fcntl.h>
// #include <sys/shm.h>




// #define BUF_SIZE 16    // two double variables
// using namespace std;
// static ofstream log_lidar("/home/sucro/log_lidar.csv");
// double speed = 0.0;    // revise
// double angle = 0.0;    // revise

// double longitude1 = 0.0; 
// double latitude1 = 0.0;
// ros::Publisher 
// struct ServerMessage
// {
//  	double longitude;
//  	double latitude;
// };
// /*
// typedef struct
// {
//         double latitude;
//         double longitude;
// }ServerMessage;

// typedef struct
// {
//         ServerMessage Servermsg;
// }TCPServerMessage;

// // TCPServerMessage tcpsevermsg;
// */
// int main(int argc, char * argv[])
// {
// 	ros::init(argc, argv, "speed_control");
// 	ros::NodeHandle nh;
        
// 	// ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64MultiArray>("/pub_l_l", 1024);

//  	// create a buffer to store the send message
//  	// char sendbuf[BUFFER_SIZE];
//  	// memset(sendbuf, 0, sizeof(sendbuf));

// 	// create a client socket cocket_client; params: ipv4\SOCK_STREAM\IPPROTO_TCP


// 	// set socket address


//        //WSADATA wsaData;

//        //WSAStartup(MAKEWORD(2, 2), &wsaData);

//        //int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

//        int sock = socket(AF_INET, SOCK_STREAM, 0);

// 	if( sock == -1)
// 	{
// 		printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
// 		exit(0);
// 	}
//         else
//         {
//              cout << "connected ...\n" << endl;  //注释3
//         }

   
//        //double buffer[BUF_SIZE] ; //文件缓冲区
//        ServerMessage buffer[16];

//        int nCount;



//         struct sockaddr_in sockAddr;

//         memset(&sockAddr, 0, sizeof(sockAddr));

//         sockAddr.sin_family = AF_INET;

//         sockAddr.sin_addr.s_addr = inet_addr("192.168.1.20");

//         sockAddr.sin_port = htons(1986);
//         int ret;
//         ret = connect(sock, (struct sockaddr*)&sockAddr, sizeof(sockAddr));

//          if (ret == -1)
//          {
//              perror("connect faile");
//              return -1;
//          }

//         printf("connect sucessfully!\n");
//         // ret = recv(sock, &buffer, sizeof(buffer), 0);
//         // printf("&&&&&&&&&&&&&&&&&&&&***&&&&&&&&&&&&&&&&&&&&&&&&\n");
	
// 	ros::Rate loop_rate(10);
// 	//while(ros::ok()){
//         ret = recv(sock, buffer, sizeof(buffer), 0);
//                 //printf("this is ret: %d", ret);

//         if (ret > 0 )	
//         {
//             longitude1 = buffer-> longitude;   //接受数据，这里应该需要接收数据的类型
//             latitude1 =  buffer-> latitude;
//         }
//         printf("this is longitude %f, and this is latitude %f\n", longitude1, latitude1);
//         while(ros::ok()){
//         std_msgs::Float64MultiArray pos;
//         pos.data[0] = longitude1;
//         pos.data[1] = latitude1;
//         chatter_pub.publish(pos);
// 	}	
//         loop_rate.sleep();
//         //}	
// 	ros::spinOnce();
// 	close(sock);
// 	return 0;
// }


#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include<geometry_msgs/Pose.h>

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




#define BUF_SIZE 16    // two double variables
using namespace std;
static ofstream log_lidar("/home/sucro/log_lidar.csv");
double speed = 0.0;    // revise
double angle = 0.0;    // revise

double longitude1 = 0.0; 
double latitude1 = 0.0;
double last_latitude = 0.0;
double last_longitude = 0.0;
ros::Publisher position_pub;
struct ServerMessage
{
 	double longitude;
 	double latitude;
};
/*
typedef struct
{
        double latitude;
        double longitude;
}ServerMessage;

typedef struct
{
        ServerMessage Servermsg;
}TCPServerMessage;

// TCPServerMessage tcpsevermsg;
*/
int main(int argc, char * argv[])
{
	ros::init(argc, argv, "speed_control");
	ros::NodeHandle nh;
    position_pub = nh.advertise<geometry_msgs::Pose>("gps_points",1);
	// ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64MultiArray>("/pub_l_l", 1024);

 	// create a buffer to store the send message
 	// char sendbuf[BUFFER_SIZE];
 	// memset(sendbuf, 0, sizeof(sendbuf));

	// create a client socket cocket_client; params: ipv4\SOCK_STREAM\IPPROTO_TCP


	// set socket address


       //WSADATA wsaData;

       //WSAStartup(MAKEWORD(2, 2), &wsaData);

       //int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

       int sock = socket(AF_INET, SOCK_STREAM, 0);

	if( sock == -1)
	{
		printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
		exit(0);
	}
        else
        {
             cout << "connected ...\n" << endl;  //注释3
        }

   
       //double buffer[BUF_SIZE] ; //文件缓冲区
       ServerMessage buffer[16];

       int nCount;

        struct sockaddr_in sockAddr;

        memset(&sockAddr, 0, sizeof(sockAddr));

        sockAddr.sin_family = AF_INET;

        sockAddr.sin_addr.s_addr = inet_addr("192.168.1.20");

        sockAddr.sin_port = htons(1986);
        int ret;
        ret = connect(sock, (struct sockaddr*)&sockAddr, sizeof(sockAddr));

        if (ret == -1)
        {
            perror("connect faile");
            return -1;
        }

        printf("connect sucessfully!\n");
        // ret = recv(sock, &buffer, sizeof(buffer), 0);
        // printf("&&&&&&&&&&&&&&&&&&&&***&&&&&&&&&&&&&&&&&&&&&&&&\n");
	
	    ros::Rate loop_rate(10);
	//while(ros::ok()){
        ret = recv(sock, buffer, sizeof(buffer), 0);
                //printf("this is ret: %d", ret);
        while(ros::ok())
        {
            if (ret > 0 )	
            {
                longitude1 = buffer-> longitude;   //接受数据，这里应该需要接收数据的类型
                latitude1 =  buffer-> latitude;
                // if(!(last_longitude==longitude1&&last_latitude==latitude1))
                {
                    geometry_msgs::Pose temp_pose;
                    temp_pose.position.x = longitude1;
                    temp_pose.position.y = latitude1;
                    position_pub.publish(temp_pose);
                    ROS_INFO("Finished pub");
                    last_latitude = latitude1;
                    last_longitude = longitude1;
                    printf("this is longitude %f, and this is latitude %f\n", longitude1, latitude1);
                }
            }

        // while(ros::ok())
        // {

        // std_msgs::Float64MultiArray pos;
        // pos.data[0] = longitude1;
        // pos.data[1] = latitude1;
        // chatter_pub.publish(pos);
	// }	
        // loop_rate.sleep();
        //}	
        }
	ros::spinOnce();
	close(sock);
	return 0;
}

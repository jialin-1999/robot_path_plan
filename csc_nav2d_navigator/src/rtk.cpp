lude <ros/ros.h> 
#include <serial/serial.h>  
#include <std_msgs/Empty.h> 
#include <std_msgs/String.h> 
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string.h>
#include <utility>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctime>
#include <fstream>

using namespace std;
serial::Serial ser; //声明串口对象
#define PI 3.1415926

ros::Publisher pub_gps;
string int2str( int val )  
{  
	ostringstream out;  
	out << val;  
	return out.str();  
} 

time_t timer=time(0);
tm* t_tm = localtime(&timer);
string save_time = "_" + int2str(t_tm->tm_year + 1900) + "_" + int2str(t_tm->tm_mon + 1) 
					+ "_" + int2str(t_tm->tm_mday) + "_" + int2str(t_tm->tm_hour) + "_" + int2str(t_tm->tm_min);
string filename ("/home/chen/dog_head/position/log");
static ofstream position_data(filename + save_time + ".csv");

 //回调函数 
void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
      ROS_INFO_STREAM("Writing to serial port" <<msg->data); //Writing to serial port
      ser.write(msg->data);   //发送串口数据 
 } 

//定义GPS变量结构体
typedef struct _GW
 {
     double Latitude;//weidu
     double Longitude;//jingdu
     double Altitude;//gaodu
 }GW;

/*dingyiyige jiegouti*/
typedef struct 
{
    char **str;     //the PChar of string array
    size_t num;     //the number of string
}IString;

// /*split the string according to ","*/
int Split(char *src, char *delim, IString* istr)    //split buf
{
    int i;
    char *str = NULL, *p = NULL;

    (*istr).num = 1;
        str = (char*)calloc(strlen(src)+1,sizeof(char));
        if (str == NULL) return 0;
    (*istr).str = (char**)calloc(1,sizeof(char *));
    if ((*istr).str == NULL) return 0;
    strcpy(str,src);

        p = strtok(str, delim);
        (*istr).str[0] = (char*)calloc(strlen(p)+1,sizeof(char));
        if ((*istr).str[0] == NULL) return 0;
        strcpy((*istr).str[0],p);
        for(i=1; p = strtok(NULL, delim); i++)
    {
        (*istr).num++;
        (*istr).str = (char**)realloc((*istr).str,(i+1)*sizeof(char *));
        if ((*istr).str == NULL) return 0;
        (*istr).str[i] = (char*)calloc(strlen(p)+1,sizeof(char));
        if ((*istr).str[0] == NULL) return 0;
        strcpy((*istr).str[i],p);
    }
    free(str);
    str = p = NULL;

    return 1;
}


int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "serial_example_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 

     //订阅主题，并配置回调函数 
      ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
    //发布主题 
      ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 
      pub_gps = nh.advertise<sensor_msgs::NavSatFix>("/fix_cxy",10);
    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port"); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    int size = 1024;
    string result;
    string buffera;
    char buffer1[360];
    IString istr;
    GW gw;

    ros::Rate loop_rate(50); 


    while(ros::ok()) 
    {     
        if(ser.available())
        {     
        ser.readline(buffera,size,"\n");

        memset(buffer1,0,sizeof(buffer1));

        strcpy(buffer1,buffera.c_str());
        
        Split(buffer1,",",&istr);

	    gw.Latitude = atof(istr.str[2]);//2
	    gw.Longitude = atof(istr.str[4]);//4
	    gw.Altitude  = atof(istr.str[9]);//9
        sensor_msgs::NavSatFix fix;
        ros::Time timestap= ros::Time::now();
        fix.header.stamp = timestap;
        
        
        fix.latitude = gw.Latitude;
        fix.longitude = gw.Longitude;
        pub_gps.publish(fix);
        
        }

		ROS_INFO("the jingdu weidu gaodu is %f , %f, %f", gw.Latitude/100, gw.Longitude/100, gw.Altitude);
      
       } 

        ros::spinOnce(); 
        loop_rate.sleep(); 
    
}


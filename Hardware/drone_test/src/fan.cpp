#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <string>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <error.h>
#include <string.h>
#include <opencv2/cudaimgproc.hpp>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
using namespace cv;
using namespace std;
#define cam_down (get_cam_num(0))
#define cam_front (get_cam_num(1))
#define PI 3.1415927

VideoCapture capture;
Mat srcImage;
//Canny边缘检测相关变量
int g_cannydot = 50;

void distinguish_fan();//识别风扇
void catch_vertex();//找出风扇的三个顶点
int get_cam_num(int cam_id);//id=0:down id=1:front
ros::Publisher fan_pub;
int img_name=1;
int catch_img=0;

void capture_img(const std_msgs::Int8 receive)
{
	catch_img=receive.data;
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "fan");
	ros::NodeHandle nh;
	fan_pub = nh.advertise<geometry_msgs::Point>("/camera_front/pose", 10);
	ros::Subscriber capture_sub = nh.subscribe<std_msgs::Int8>("/camera_front/capture", 10, capture_img);
	capture.open(cam_front,CAP_V4L2);//打开摄像头
	if (!capture.isOpened())//如果视频不能正常打开则返回
	{
		cout << "摄像头打开失败！" << endl;
		return 1;
	}

	while (1)
	{
		capture >> srcImage;//等价于 capture.read(frame);
		if (srcImage.empty())//如果某帧为空则退出循环
		{
			cout << "摄像头断开！" << endl;
			break;
		}
		if(catch_img==2)
		{
			catch_vertex();
			catch_img=0;
			continue;
		}
		distinguish_fan();//识别风扇
		ros::spinOnce();

		waitKey(1);//每帧延时 1 毫秒，如果不延时，图像将无法显示
	}

	capture.release();//释放资源
	return 0;
}

int get_cam_num(int cam_id)//id=0:down id=1:front
{
	struct v4l2_capability cap;
	bool exist_0=true;
	bool exist_1=true;
	int num[2]={-1,-1};
	int fd=open("/dev/video0",O_RDWR);
	if(fd<0)exist_0=false;
	else
	{
		int ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
		if (ret < 0) 
		{
			perror("ioctl querycap");
			return -1;
		}
		num[0]=cap.bus_info[strlen((const char *)cap.bus_info)-1]-'3';
		//cout<<cap.bus_info<<endl;
		close(fd);
	}
	fd=open("/dev/video1",O_RDWR);
	if(fd<0)exist_1=false;
	else
	{
		int ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
		if (ret < 0) 
		{
			perror("ioctl querycap");
			return -1;
		}
		//cout<<cap.bus_info<<endl;
		num[1]=cap.bus_info[strlen((const char *)cap.bus_info)-1]-'3';
		close(fd);
	}
	int id[2]={-1,-1};
	if((!exist_0)&&(!exist_1)) return -1;
	else if(exist_0&&(!exist_1))return 0;
	else if((!exist_0)&&exist_1)return 1;
	if(num[0]==1)
	{
		id[1]=0;
		id[0]=1;
	}
	else
	{
		id[0]=0;
		id[1]=1;
	}
	return id[cam_id];
}

//最大轮廓检测
void distinguish_fan()
{
	resize(srcImage, srcImage, Size(320, 240), 0, 0, INTER_AREA);
	//创建显示窗口和滑动条
	namedWindow("dot");
	createTrackbar("g_cannydot", "canny", &g_cannydot, 100);

	// 将原图像转换为灰度图像
	Mat grayImage, imgCanny;
	Mat element;
	cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
	//腐蚀
	element = getStructuringElement(MORPH_RECT, Size(7, 7));
	morphologyEx(grayImage, grayImage, MORPH_ERODE, element);
	Canny(grayImage, imgCanny, g_cannydot, 3 * g_cannydot, 3);//进行一次canny边缘检测
	//检测轮廓
	vector<vector<Point>> contours;
	findContours(imgCanny, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	//寻找最大轮廓
	int i, k;
	double length_max = 0;
	if (contours.size()>0)
	{
		k = 0;
		for (i = 0; i<contours.size(); i++)
		{

			if (arcLength(contours[i], 0)>length_max)
			{
				length_max = arcLength(contours[i], 0);
				k = i;
			}
		}
		//绘制最大轮廓
		drawContours(srcImage, contours, k, Scalar(122, 135, 255), 4);
		//计算最大轮廓中心点坐标
		Point2f vertices[4];
		Point2f center_box;
		RotatedRect box = minAreaRect(contours[k]);//寻找最小包围矩形
		box.points(vertices);
		//计算中心点坐标
		center_box.x = (vertices[0].x + vertices[2].x) / 2.0;
		center_box.y = (vertices[0].y + vertices[2].y) / 2.0;
		circle(srcImage, center_box, 3, Scalar(0, 255, 0), -1, 8, 0);
		geometry_msgs::Point pose;
		pose.x=160-center_box.x;
		pose.y=120-center_box.y;
		pose.z=0;
		fan_pub.publish(pose);
		
		//绘制最小面积包围矩形
		for (int i = 0; i < 4; i++)
			line(srcImage, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 2, 8);
	}

	imshow("canny", imgCanny);
	imshow("dot", srcImage);
}

//找出风扇的三个顶点
void catch_vertex()
{
	Mat grayImage, imgCanny;
	cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
	Canny(grayImage, imgCanny, 70, 210, 3);//进行一次canny边缘检测
	int row = srcImage.rows;	//获取矩阵的行
	int nStep = imgCanny.cols * imgCanny.channels();//计算总共需要遍历的像素点的个数
	Point target;
	int first=1;
	for (int i = 0; i < row; i++)
	{
		uchar *Data = imgCanny.ptr<uchar>(i);
		for (int j = 0; j < nStep; j++)
		{
			if(Data[j]==255)
			{
				if(first==1)
				{
					first=0;
					target.x=j;
					target.y=i;
				}
				else if(img_name==1 && i<target.y)
				{
					target.x=j;
					target.y=i;
				}
				else if(img_name==2 && j<target.x)
				{
					target.x=j;
					target.y=i;
				}
				else if(img_name==3 && j>target.x)
				{
					target.x=j;
					target.y=i;
				}
			}
		}
	}
	Point left_above;
	left_above.x=target.y-100;
	left_above.y=target.y+100;
	Point right_down;
	right_down.x=target.x-100;
	right_down.y=target.x+100;
	if(left_above.x<0)
		left_above.x=0;
	if(left_above.y>480)
		left_above.y=480;
	if(right_down.x<0)
		right_down.x=0;
	if(right_down.y>640)
		right_down.y=640;
	Mat ROIimage;
	ROIimage = srcImage(Range(left_above.x,left_above.y),Range(right_down.x,right_down.y));
	//存储照片
	string name=to_string(img_name++)+".jpg";
	imwrite(name,ROIimage);
}
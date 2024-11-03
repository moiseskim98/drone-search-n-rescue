#include<opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <linux/videodev2.h>
#include<sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <error.h>
#include <string.h>
#include <opencv2/cudaimgproc.hpp>
#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <vector>

using namespace cv;
using namespace std;
#define cam_down (get_cam_num(0))
#define cam_front (get_cam_num(1))
Mat srcImage;
Mat grayImage;
//Canny边缘检测相关变量
int g_canny_hough = 90;
int g_houghcircles1 = 14;
int g_houghcircles2 = 66;
int g_houghcircles3 = 35;
//Canny边缘检测相关变量
int g_canny_line = 50;
int g_houghlines1 = 80;
int g_houghlines2 = 50;
int g_houghlines3 = 10;

double confidence_circle=0;
double confidence_line=0;



ros::Subscriber mode_sub;
ros::Publisher pos_pub;
ros::Publisher line_pub;
int mode=0;
void common_processing();
bool huofucircles();//定点（霍夫圆检测）
bool huofulines();

void cal_confidence_circle(bool status)
{
	static vector<bool> circle_count;
	circle_count.push_back(status);
	if(circle_count.size()>30)circle_count.erase(circle_count.begin());
	double count=0;
	for(int i=0;i<circle_count.size();i++)
    {
         count+=circle_count[i];
    } 
	if(circle_count.size()!=0)
	{
		confidence_circle=count/circle_count.size();
	}
}

void cal_confidence_line(bool status)
{
	static vector<bool> line_count;
	line_count.push_back(status);
	if(line_count.size()>30)line_count.erase(line_count.begin());
	double count=0;
	for(int i=0;i<line_count.size();i++)
    {
         count+=line_count[i];
    } 
	if(line_count.size()!=0)
	{
		confidence_line=count/line_count.size();
	}
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

void mode_cb(const std_msgs::Int8::ConstPtr& msg)
{
  mode = msg->data;
}



int main(int argc, char** argv) 
{
	VideoCapture capture(cam_down,CAP_V4L2);

	if (!capture.isOpened())//如果视频不能正常打开则返回
	{
		cout << "摄像头打开失败！" << endl;
		return -1;
	}
	ros::init(argc, argv, "camera_down_node");
	ros::NodeHandle nh;
	pos_pub = nh.advertise<geometry_msgs::Point>("/camera_down/pose", 10);
	line_pub = nh.advertise<geometry_msgs::Point>("/camera_down/line", 10);
	mode_sub = nh.subscribe<std_msgs::Int8>("/camera_down/mode", 10, mode_cb);
	ros::Publisher confidence_circle_pub=nh.advertise<std_msgs::Float64>("/camera_down/pose_confi", 10);
	ros::Publisher confidence_line_pub=nh.advertise<std_msgs::Float64>("/camera_down/line_confi", 10); 
	while (ros::ok())
	{
		double fps,t=0;
		
		t = (double)getTickCount();
		capture >> srcImage;//等价于 capture.read(frame);
		if (srcImage.empty())//如果某帧为空则退出循环
		{
			cout << "摄像头断开！" << endl;
			break;
		}
		common_processing();
		if(mode&0x01)
		{
			std_msgs::Float64 confi_cir;
			bool status=huofucircles();//定点（霍夫圆检测）
			cal_confidence_circle(status);
//			cout<<"circle_confidence="<<confidence_circle<<endl;
			confi_cir.data=confidence_circle;
			confidence_circle_pub.publish(confi_cir);
		}
		if(mode&0x02)
		{
			std_msgs::Float64 confi_line;
			bool status=huofulines();
			cal_confidence_line(status);
//			cout<<"line_confidence="<<confidence_line<<endl;
			confi_line.data=confidence_line;
			confidence_line_pub.publish(confi_line);			
		}

		ros::spinOnce();
		waitKey(1);//每帧延时 1 毫秒，如果不延时，图像将无法显示
		t = ((double)getTickCount() - t) / getTickFrequency();
		fps = 1.0 / t;
		cout<<"fps:"<<fps<<endl;
	}

	capture.release();//释放资源
	return 0;
}

//定点（霍夫圆检测）
bool huofucircles()
{
	bool status=false;
	// 创建显示窗口
	namedWindow("Circle", WINDOW_AUTOSIZE);
	// 创建滑动条
	createTrackbar("canny", "Circle", &g_canny_hough, 100);
	createTrackbar("最小半径：", "Circle", &g_houghcircles1, 300);
	createTrackbar("最大半径", "Circle", &g_houghcircles2, 1000);
	createTrackbar("累加器阈值", "Circle", &g_houghcircles3, 300);

	// 将原图像转换为灰度图像
	Mat midImage, dstImage;
	srcImage.copyTo(dstImage);
	Canny(grayImage, midImage, g_canny_hough, 3 * g_canny_hough, 3);//进行一次canny边缘检测
	cv::cuda::GpuMat gray_gpu;
	cv::cuda::GpuMat d_circles;
	gray_gpu.upload(midImage);
	cv::Ptr<cv::cuda::HoughCirclesDetector> houghCircles = cv::cuda::createHoughCirclesDetector(1.5f, 100, 100, g_houghcircles3, g_houghcircles1, g_houghcircles2);
	houghCircles->detect(gray_gpu, d_circles);
	
	//进行霍夫圆变换
	vector<Vec3f> circles;
//	HoughCircles(midImage, circles, HOUGH_GRADIENT, 1.5, 100, 100, g_houghcircles3, g_houghcircles1, g_houghcircles2);
	circles.resize(d_circles.size().width);
	if (!circles.empty())
		d_circles.row(0).download(cv::Mat(circles).reshape(3, 1));
//	cout<<"count:"<<circles.size()<<endl;


	int k = 0, send_x = 0, send_y = 0;
	//依次在图中绘制出圆
	for (size_t i = 0; i < circles.size(); i++)
	{
		//参数定义
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		if (cvRound(circles[k][2])<cvRound(circles[i][2]))
		{
			k = i;
		}
		//绘制圆心
		circle(dstImage, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		//绘制圆轮廓
		circle(dstImage, center, radius, Scalar(155, 50, 255), 2, 8, 0);
		//printf("%d,%d\n",center.x,center.y);
	}
	if (circles.size() != 0)
	{
		send_x = cvRound(circles[k][0]);
		send_y = cvRound(circles[k][1]);
		if (send_x != 0 && send_y != 0)
		{
			//发送数据
//			cout << 120 - send_y << "," << 160 - send_x << endl;
			geometry_msgs::Point pose;
			pose.x=120 - send_y;
			pose.y=160 - send_x;
			pose.z=0;
			pos_pub.publish(pose);
			status=true;
		}
	}

	imshow("Canny", midImage);
	imshow("Circle", dstImage);
	return status;
}


//巡线（霍夫线变换）
bool huofulines()
{
	bool status=false;
	//创建窗口和滑动条
	namedWindow("line");
	createTrackbar("canny", "Canny", &g_canny_line, 100);
	createTrackbar("累加平面的阈值参数（巡线）", "line", &g_houghlines1, 300);
	createTrackbar("最短线段长度", "line", &g_houghlines2, 250);
	createTrackbar("允许连接的最大距离", "line", &g_houghlines3, 250);
	Mat midImage, dstImage;
	srcImage.copyTo(dstImage);
	Canny(grayImage, midImage, g_canny_line, g_canny_line * 3, 3);
	//进行霍夫线变换
	vector<Vec4i> lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
	HoughLinesP(midImage, lines, 1, CV_PI / 180, g_houghlines1, g_houghlines2, g_houghlines3);

	double ratios = 0, intercepts = 0;
	double ratiomedium, interceptmedium;
	vector<Point2f> points_to_fit;
	//将竖直方向的线存入数组中
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		float ratio;
		float intercept;
		//忽略水平的线
		if (abs(l[0] - l[2]) < abs(l[1] - l[3]))
		{
			line(dstImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, LINE_AA);
			//计算斜率和横截距
			ratio = (l[0] - l[2])*1.0 / (l[1] - l[3]);
			intercept = l[0] * 1.0 - ratio*l[1];
			points_to_fit.push_back(Point2f(ratio, intercept));
		}
	}
	//求取均值
	if (points_to_fit.size()>0)
	{
		for (size_t i = 0; i<points_to_fit.size(); i++)
		{
			ratios += points_to_fit[i].x;
			intercepts += points_to_fit[i].y;
		}
		ratiomedium = ratios*1.0 / points_to_fit.size();
		interceptmedium = intercepts*1.0 / points_to_fit.size();
		//printf("%.2f,%.2f\n",ratiomedium,interceptmedium);
		double theta;
		double rho;
		rho = -ratiomedium*interceptmedium / fabs(ratiomedium) / sqrt(ratiomedium*ratiomedium + 1);
		theta = M_PI / 2 + atan(1.0 / ratiomedium);
		if (ratiomedium<0)
		{
			line(dstImage, Point2f(interceptmedium, 0), Point2f(0, -interceptmedium*1.0 / ratiomedium), Scalar(186, 88, 255), 5, LINE_AA);
		}
		else if (ratiomedium>0)
		{
			line(dstImage, Point2f(interceptmedium, 0), Point2f( 320, ((320 - interceptmedium)*1.0 / ratiomedium)), Scalar(186, 88, 255), 5, LINE_AA);
		}
		else
		{
			rho = interceptmedium;
			theta = 0;
			line(dstImage, Point2f(interceptmedium, 0), Point2f(interceptmedium, 480), Scalar(186, 88, 255), 5, LINE_AA);
		}
		//发送数据
		geometry_msgs::Point pose;

		if (rho<0)
		{
			//printf("%.2f,%.2f\n",(theta-PI)/PI*180,fabs(rho)-160);
			pose.x=M_PI-theta;
			pose.y=160-fabs(rho);
			pose.z=0;

		}
		else
		{
			//printf("%.2f,%.2f\n",theta/PI*180,fabs(rho)-160);
			pose.x=-theta;
			pose.y=160-fabs(rho);
			pose.z=0;
		}
		line_pub.publish(pose);
		status=true;
	}
	else
	{
		//如果没有识别到线
	}
	imshow("Canny1", midImage);
	imshow("line", dstImage);
	return status;
}

void common_processing()
{
	resize(srcImage, srcImage, Size(320, 240), 0, 0, INTER_AREA);
		//进行边缘检测
	cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
}
#include<opencv2/opencv.hpp>
#include <iostream>

#define PI 3.1415927

using namespace cv;
using namespace std;

VideoCapture capture;
Mat srcImage;
//Canny边缘检测相关变量
int g_canny_line = 50;
int g_houghlines1 = 80;
int g_houghlines2 = 50;
int g_houghlines3 = 10;

void huofulines();//巡线（霍夫线变换）

int main() 
{
	capture.open(0); //打开摄像头
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

		huofulines();

		waitKey(1);//每帧延时 1 毫秒，如果不延时，图像将无法显示
	}

	capture.release();//释放资源
	return 0;
}

//巡线（霍夫线变换）
void huofulines()
{
	resize(srcImage, srcImage, Size(320, 240), 0, 0, INTER_AREA);

	//创建窗口和滑动条
	namedWindow("line");
	createTrackbar("canny", "Canny", &g_canny_line, 100);
	createTrackbar("累加平面的阈值参数（巡线）", "line", &g_houghlines1, 300);
	createTrackbar("最短线段长度", "line", &g_houghlines2, 250);
	createTrackbar("允许连接的最大距离", "line", &g_houghlines3, 250);

	Mat grayImage, midImage, dstImage;
	//进行边缘检测
	cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
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
			line(srcImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, LINE_AA);
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
		theta = PI / 2 + atan(1.0 / ratiomedium);
		if (ratiomedium<0)
		{
			line(srcImage, Point2f(interceptmedium, 0), Point2f(0, -interceptmedium*1.0 / ratiomedium), Scalar(186, 88, 255), 5, LINE_AA);
		}
		else if (ratiomedium>0)
		{
			line(srcImage, Point2f(interceptmedium, 0), Point2f( 320, ((320 - interceptmedium)*1.0 / ratiomedium)), Scalar(186, 88, 255), 5, LINE_AA);
		}
		else
		{
			rho = interceptmedium;
			theta = 0;
			line(srcImage, Point2f(interceptmedium, 0), Point2f(interceptmedium, 480), Scalar(186, 88, 255), 5, LINE_AA);
		}
		//发送数据
		if (rho<0)
		{
			printf("%.2f,%.2f\n",(theta-PI)/PI*180,fabs(rho)-160);
		}
		else
		{
			printf("%.2f,%.2f\n",theta/PI*180,fabs(rho)-160);
		}
	}
	else
	{
		//如果没有识别到线
	}
	imshow("Canny", midImage);
	imshow("line", srcImage);
}
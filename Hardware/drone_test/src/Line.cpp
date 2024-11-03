#include<opencv2/opencv.hpp>
#include <iostream>

#define PI 3.1415927

using namespace cv;
using namespace std;

VideoCapture capture;
Mat srcImage;
//Canny��Ե�����ر���
int g_canny_line = 50;
int g_houghlines1 = 80;
int g_houghlines2 = 50;
int g_houghlines3 = 10;

void huofulines();//Ѳ�ߣ������߱任��

int main() 
{
	capture.open(0); //������ͷ
	if (!capture.isOpened())//�����Ƶ�����������򷵻�
	{
		cout << "����ͷ��ʧ�ܣ�" << endl;
		return 1;
	}

	while (1)
	{
		capture >> srcImage;//�ȼ��� capture.read(frame);
		if (srcImage.empty())//���ĳ֡Ϊ�����˳�ѭ��
		{
			cout << "����ͷ�Ͽ���" << endl;
			break;
		}

		huofulines();

		waitKey(1);//ÿ֡��ʱ 1 ���룬�������ʱ��ͼ���޷���ʾ
	}

	capture.release();//�ͷ���Դ
	return 0;
}

//Ѳ�ߣ������߱任��
void huofulines()
{
	resize(srcImage, srcImage, Size(320, 240), 0, 0, INTER_AREA);

	//�������ںͻ�����
	namedWindow("line");
	createTrackbar("canny", "Canny", &g_canny_line, 100);
	createTrackbar("�ۼ�ƽ�����ֵ������Ѳ�ߣ�", "line", &g_houghlines1, 300);
	createTrackbar("����߶γ���", "line", &g_houghlines2, 250);
	createTrackbar("�������ӵ�������", "line", &g_houghlines3, 250);

	Mat grayImage, midImage, dstImage;
	//���б�Ե���
	cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
	Canny(grayImage, midImage, g_canny_line, g_canny_line * 3, 3);
	//���л����߱任
	vector<Vec4i> lines;//����һ��ʸ���ṹlines���ڴ�ŵõ����߶�ʸ������
	HoughLinesP(midImage, lines, 1, CV_PI / 180, g_houghlines1, g_houghlines2, g_houghlines3);

	double ratios = 0, intercepts = 0;
	double ratiomedium, interceptmedium;
	vector<Point2f> points_to_fit;
	//����ֱ������ߴ���������
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		float ratio;
		float intercept;
		//����ˮƽ����
		if (abs(l[0] - l[2]) < abs(l[1] - l[3]))
		{
			line(srcImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, LINE_AA);
			//����б�ʺͺ�ؾ�
			ratio = (l[0] - l[2])*1.0 / (l[1] - l[3]);
			intercept = l[0] * 1.0 - ratio*l[1];
			points_to_fit.push_back(Point2f(ratio, intercept));
		}
	}
	//��ȡ��ֵ
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
		//��������
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
		//���û��ʶ����
	}
	imshow("Canny", midImage);
	imshow("line", srcImage);
}
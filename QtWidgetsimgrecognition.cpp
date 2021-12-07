#include "QtWidgetsimgrecognition.h"
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <sstream>    // ostringstream 类型
#include <strstream>
#include <Qtimer>


cv::Mat ExtractDepthImage(open3d::geometry::PointCloud save_cloud,cv::Mat save_image,double num) {
	
	cv::Mat gray;
	//cv::Mat img_copy;
	cv::Mat g_srcImage = save_image;
	//cv::Mat g_dstImage = cv::Mat::zeros(g_srcImage.size(), g_srcImage.type());
	double compareValue = 4;
	double lightValue = 50;
	//g_srcImage.convertTo(g_dstImage, -1, compareValue, lightValue);
	//img_copy = g_dstImage.clone();
	cv::cvtColor(save_image, gray, cv::COLOR_BGR2GRAY);
	for (int r = 0; r < save_image.rows; r++)
	{
		for (int c = 0; c < save_image.cols; c++)
		{
			if (isnan(save_cloud.points_[save_image.cols*r + c].z())|| save_cloud.points_[save_image.cols*r + c].z()>(num/1000))
			{
				gray.at<uchar>(r, c) = 0;
			}
			else
			{
				gray.at<uchar>(r,c)=save_cloud.points_[save_image.cols*r + c].z()*200;
			}
			
		}
	}
	return gray;
}
string ItoB(int num)
{
	string str;
	int Inum = num;
	for (int i = 15; i >= 0; i--)
	{
		if (Inum >= pow(2, i))
		{
			str += "1";
			Inum = Inum - pow(2, i);
		}
		else
		{
			str += "0";
		}
	}
	return str;
}
double BtoD(string x)
{
	double ans;
	int E = 0;
	double D = 0;
	for (int i = 1; i < 32; i++)
	{
		if (i < 9)
		{
			E += (x[i] - '0') << (8 - i);
			//cout << E << endl;
		}
		else
		{
			D += (x[i] - '0') * pow(2, (8 - i));
			//cout << D << endl;
		}

	}

	ans = pow(2, E - 127)*(1 + D);

	if (x[0] == '1')
	{
		ans = -ans;
	}
	return ans;
}
void BtoReal(string x, int output[2])
{
	output[0] = 0;
	output[1] = 0;
	for (int i = 0; i < 32; i++)
	{
		if (i < 16)
		{
			if (x[i] == '1')
			{
				output[0] += pow(2, 15-i);
			}
		}
		else
		{
			if (x[i] == '1')
			{
				output[1] += pow(2, 31-i);
			}
		}
	}
	
}
vector<bool> zhengshu;//存整数部分的二进制
vector<bool> xiaoshu;//存小数部分的二进制
vector<bool> get_zhengshu_2b(float a)
{
	vector<bool> x;
	x.clear();
	//八位二进制a xxxx xxxx与1000 0000与，得到每位的二进制数
	for (int i = 0; i < 8; i++)
	{
		if ((((int)a)&(0x80 >> i)))
		{
			x.push_back(1);
		}
		else
		{
			x.push_back(0);
		}
	}
	return x;
}

void get_2b(float a)
{
	//获取整数部分的二进制码
	float fabs_a = fabs(a);//取绝对值
	zhengshu.clear();
	xiaoshu.clear();
	zhengshu = get_zhengshu_2b(fabs_a);

	//获取小数部分的二进制码
	float n = 2;   //小数位的阶数取倒数
	float b = (fabs_a - floor(fabs_a));

	//每次除以2判断该位是0还是1
	int num = 0;
	while (num<10)
	{
		num++;
		if ((1.0 / n) < b)
		{
			xiaoshu.push_back(1);
			//若为1则b减去该位所对应的十进制小数大小 ，继续判断低一位，直到b=0
			b = b - (1.0 / n);
		}
		else if ((1.0 / n) > b)
		{
			xiaoshu.push_back(0);
		}
		else if ((1.0 / n) == b)
		{
			xiaoshu.push_back(1);
			break;
		}
		n = n * 2;
	}
}
int get_jiema()  //返回阶码
{
	for (int i = 0; i < 8; i++)
	{
		if (zhengshu[i] == 1)//判断从左边起第一个为1的位置
			return 7 - i;		// 返回阶码大小
	}
	for (int j = 0; j < xiaoshu.size(); j++)
	{
		if (xiaoshu[j] == 1)
			return -(j+1);
	}
}
vector<bool> get_yima()//得到移码
{
	int e = get_jiema();
	e = e + 127;  //阶码偏移得到移码
	return get_zhengshu_2b(e);//返回获得的移码的二进制形式
}
vector<bool> get_weima()//获得尾码
{
	vector <bool> m;
	//小数的二进制前插入规格化的码得到尾码
	if (get_jiema() >=0)
	{
		xiaoshu.insert(xiaoshu.begin(), zhengshu.begin() + (8 - get_jiema()), zhengshu.end());
	}
	else
	{
		//xiaoshu.insert(xiaoshu.begin(), xiaoshu.begin()+abs(get_jiema()) , zhengshu.end());
		xiaoshu.erase(xiaoshu.begin(), xiaoshu.begin()+abs(get_jiema()));
	}
	m = xiaoshu;
	return m;
}
string to_IEEE754(float x)
{
	zhengshu.clear();
	xiaoshu.clear();
	vector<bool> IEEE;
	IEEE.clear();
	get_2b(x);   //得到x的二进制码
	
	//组合成短浮点数代码：
	vector<bool> yima;
	yima.clear();
	yima = get_yima();
	vector<bool> weima;
	weima.clear();
	weima = get_weima();

	if (x > 0)//判断并添加符号位
	{
		IEEE.insert(IEEE.end(), 1, 0);
	}
	else
	{
		IEEE.insert(IEEE.end(), 1, 1);
	}
	IEEE.insert(IEEE.end(), yima.begin(), yima.end());//添加移码
	IEEE.insert(IEEE.end(), weima.begin(), weima.end());//添加尾码
	IEEE.insert(IEEE.end(), 32 - 9 - weima.size(), 0);//尾部补零 共32位
	string str;
	for (auto it : IEEE)
	{
		str += to_string(it);
	}
	return str;
}
QtWidgetsimgrecognition::QtWidgetsimgrecognition(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	ui.cameratype->setStyleSheet(QString::fromStdString("color:red"));
	connect(ui.readimg, SIGNAL(clicked(bool)), this, SLOT(readimg()));
	connect(ui.comp, SIGNAL(valueChanged(int)), this, SLOT(updataimg()));
	connect(ui.light, SIGNAL(valueChanged(int)), this, SLOT(updataimg()));
	connect(ui.recognition, SIGNAL(clicked(bool)), this, SLOT(recognitionTarget()));
	connect(ui.recognitionleg, SIGNAL(clicked(bool)), this, SLOT(recognitionLeg()));
	connect(ui.canny, SIGNAL(clicked(bool)), this, SLOT(cannyMod()));
	connect(ui.showCut, SIGNAL(clicked(bool)), this, SLOT(ShowCutPoint()));
	connect(ui.connectcamera, SIGNAL(clicked(bool)), this, SLOT(camerainit()));
	connect(ui.disconnectCamera, SIGNAL(clicked(bool)), this, SLOT(cameraclose()));
	connect(ui.cameraopen, SIGNAL(clicked(bool)), this, SLOT(cameraOpen()));
	connect(ui.readcloud, SIGNAL(clicked(bool)), this, SLOT(readcloud()));
	connect(ui.RGB, SIGNAL(toggled(bool)), this, SLOT(updataimg()));
	connect(ui.gray, SIGNAL(toggled(bool)), this, SLOT(updataimg()));
	connect(ui.depthImg, SIGNAL(toggled(bool)), this, SLOT(updataimg()));
	connect(ui.TCP, SIGNAL(clicked(bool)), this, SLOT(readRobotPose()));
	connect(ui.readeyetohand, SIGNAL(clicked(bool)), this, SLOT(readEyetohand()));
	connect(ui.readSampleRobotPose, SIGNAL(clicked(bool)), this, SLOT(readSampleRobot()));
	connect(ui.readSampleRobotPose, SIGNAL(clicked(bool)), this, SLOT(readErrorPoint()));
	connect(ui.saveSampleImg, SIGNAL(clicked(bool)), this, SLOT(saveOnePoint()));
	connect(ui.saveSample, SIGNAL(clicked(bool)), this, SLOT(saveSample()));
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(scanPLC()));
	connect(ui.scanOpen, SIGNAL(clicked(bool)), this, SLOT(ScanOpen()));
	connect(ui.scanClose, SIGNAL(clicked(bool)), this, SLOT(ScanClose()));
	connect(ui.samplelines, SIGNAL(clicked(bool)), this, SLOT(save_2D_lines()));
	connect(ui.readlines, SIGNAL(clicked(bool)), this, SLOT(read_2D_lines()));
	connect(ui.counterror, SIGNAL(clicked(bool)), this, SLOT(cutCloud()));
	connect(ui.settingPage, SIGNAL(clicked(bool)), this, SLOT(SettingPage()));
	connect(ui.MonitorPage, SIGNAL(clicked(bool)), this, SLOT(MonitorPage()));
	Sampleinit();
}
void QtWidgetsimgrecognition::SettingPage()
{
	ui.stackedWidget->setCurrentIndex(1);
}
void QtWidgetsimgrecognition::MonitorPage()
{
	ui.stackedWidget->setCurrentIndex(0);
}
void QtWidgetsimgrecognition::ScanOpen()
{
	timer->start(2000);
}
void QtWidgetsimgrecognition::ScanClose()
{

	timer->stop();
}
void QtWidgetsimgrecognition::readimg()
{
	filename = QFileDialog::getOpenFileName(this,
		tr("open image"),
		".",
		tr("Image file(*.png *.jpg *.bmp)"));
	cv::Mat gray;
	cv::Mat img_copy;
	cv::Mat image= cv::imread(filename.toStdString());
	save_image = image;
	updataimg();
}
void QtWidgetsimgrecognition::showimg()
{
	cv::Mat RGB;
	cv::Mat g_srcImage = save_image;
	cv::Mat g_dstImage = cv::Mat::zeros(g_srcImage.size(), g_srcImage.type());
	double compareValue = ui.comp->value() / 100;
	double lightValue = ui.light->value();
	g_srcImage.convertTo(g_dstImage, -1, compareValue, lightValue);
	cv::cvtColor(g_dstImage, RGB,cv::COLOR_BGR2RGB);
	QImage img = QImage((const unsigned char*)(RGB.data),
		g_dstImage.cols, g_dstImage.rows,g_dstImage.step, QImage::Format_RGB888);
	//设定图像大小自适应label窗口的大小
	img = img.scaled(ui.label->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	QPixmap pixmap = QPixmap::fromImage(img);
	ui.label->setPixmap(pixmap);
}
void QtWidgetsimgrecognition::showgray()
{
	cv::Mat gray;
	cv::Mat img_copy;
	cv::Mat g_srcImage = save_image;
	cv::Mat g_dstImage = cv::Mat::zeros(g_srcImage.size(), g_srcImage.type());
	double compareValue = ui.comp->value() / 100;
	double lightValue = ui.light->value();
	g_srcImage.convertTo(g_dstImage, -1, compareValue, lightValue);
	img_copy = g_dstImage.clone();
	cv::cvtColor(img_copy, gray, cv::COLOR_BGR2GRAY);
	QImage img = QImage((const unsigned char*)(gray.data),
		g_dstImage.cols, g_dstImage.rows, QImage::Format_Grayscale8);
	//设定图像大小自适应label窗口的大小
	img = img.scaled(ui.label->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	QPixmap pixmap = QPixmap::fromImage(img);
	ui.label->setPixmap(pixmap);
}
void QtWidgetsimgrecognition::showdepth()
{
	QImage img = QImage((const unsigned char*)(depth_image.data),
		depth_image.cols, depth_image.rows, QImage::Format_Grayscale8);
	//设定图像大小自适应label窗口的大小
	img = img.scaled(ui.label->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	QPixmap pixmap = QPixmap::fromImage(img);
	ui.label->setPixmap(pixmap);

}
void QtWidgetsimgrecognition::updataimg()
{
	if (save_image.rows < 1)
	{
		return;
	}
	
	if (ui.RGB->isChecked())
	{
			showimg();
	}
	if (ui.gray->isChecked())
	{
			showgray();
	}	
	if (ui.depthImg->isChecked())
	{
		if (depth_image.rows > 0)
		{
			showdepth();
		}
	}
	int cor[4];
	ui.label->getXY(cor);
	ui.x_first->setText(QString::fromStdString(std::to_string(cor[0])));
	ui.y_first->setText(QString::fromStdString(std::to_string(cor[1])));
	ui.x_final->setText(QString::fromStdString(std::to_string(cor[2])));
	ui.y_final->setText(QString::fromStdString(std::to_string(cor[3])));
}
void QtWidgetsimgrecognition::readcloud()
{
	QString path = QFileDialog::getOpenFileName(this,
		tr("open image"),
		".",
		tr("Image file(*.ply)"));
	open3d::io::ReadPointCloudFromPLY(path.toStdString(), save_cloud);
	auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
	cloud_ptr->points_ = save_cloud.points_;
	Eigen::Vector3d color;
	color[0] = 1;
	color[1] = 0;
	color[2] = 0;
	cloud_ptr->PaintUniformColor(color);
	if (save_cloud.points_.size() > 1)
	{
		depth_image = ExtractDepthImage(save_cloud, save_image, ui.height->value());
	}
	//cloud_ptr->VoxelDownSample(0.01);
	//open3d::visualization::DrawGeometries({ cloud_ptr });
}
void QtWidgetsimgrecognition::recognitionTarget()
{
	cv::Mat gray;
	cv::Mat img_copy;
	//cv::Mat image = cv::imread(filename.toStdString());
	cv::Mat g_srcImage = save_image;
	cv::Mat g_dstImage = cv::Mat::zeros(g_srcImage.size(), g_srcImage.type());
	QImage img;
	int cor[4];
	std::vector<cv::Point2d> point_group;
	ui.label->getXY(cor);
	int wigth = (cor[2] - cor[0]) * 2;
	int high = (cor[3] - cor[1]) * 2;
	ui.x_first->setText(QString::fromStdString(std::to_string(cor[0])));
	ui.y_first->setText(QString::fromStdString(std::to_string(cor[1])));
	ui.x_final->setText(QString::fromStdString(std::to_string(cor[2])));
	ui.y_final->setText(QString::fromStdString(std::to_string(cor[3])));
	if (save_image.size().height > 1)
	{
		double compareValue = ui.comp->value() / 100;
		double lightValue = ui.light->value();
		g_srcImage.convertTo(g_dstImage, -1, compareValue, lightValue);
		img_copy = g_dstImage.clone();
		cv::cvtColor(img_copy, gray, cv::COLOR_BGR2GRAY);
		img = QImage((const unsigned char*)(gray.data),
			g_dstImage.cols, g_dstImage.rows, QImage::Format_Grayscale8);
		//设定图像大小自适应label窗口的大小
		img = img.scaled(ui.label->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
		//
	}
	point_group = CutImg(gray, cor);
	if (point_group.size() < wigth * 2)
	{
		for (int i = -30; i < 30; i += 10)
		{
			for (int j = -30; j < 30; j += 10)
			{
				double compareValue = (ui.comp->value() + i) / 100;
				double lightValue = ui.light->value() + j;
				g_srcImage.convertTo(g_dstImage, -1, compareValue, lightValue);
				img_copy = g_dstImage.clone();
				cv::cvtColor(img_copy, gray, cv::COLOR_BGR2GRAY);
				img = QImage((const unsigned char*)(gray.data),
					g_dstImage.cols, g_dstImage.rows, QImage::Format_Grayscale8);
				//设定图像大小自适应label窗口的大小
				img = img.scaled(ui.label->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
				point_group = CutImg(gray, cor);
				if (point_group.size() == wigth * 2)
				{
					break;
				}
			}
			if (point_group.size() == wigth * 2)
			{
				break;
			}
		}
	}
	if (point_group.size() < wigth * 2)
	{
		return;
	}
	QPixmap pixmap = QPixmap::fromImage(img);
	QPainter painter(&pixmap);
	painter.setPen(QPen(Qt::green, 2));
	painter.setRenderHint(QPainter::Antialiasing, true);  //设置渲染,启动反锯齿

	for (auto it : point_group)
	{
		double x = cor[0] + it.x / 2 - 0.5;
		double y = cor[1] + it.y / 2 - 0.5;
		painter.drawEllipse(x, y, 1, 1);
	}
	//半径为50的圆
	//painter.drawEllipse(0, 0, 100, 100);
	//read pointcloud//
	open3d::geometry::PointCloud source;
	source = save_cloud;
	std::vector<Eigen::Vector3d> group = cut3DMod(source);
	open3d::geometry::PointCloud cloud;
	auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
	cloud_ptr->points_ = group;
	cloud_ptr->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(200));
	//              //
	std::vector<Eigen::Vector3d> color;
	for (auto it : group)
	{
		Eigen::Vector3d p_c;
		p_c[0] = 0;
		p_c[1] = 1;
		p_c[2] = 0;
		color.push_back(p_c);
	}
	cloud_ptr->colors_ = color;

	std::vector<cv::Point2d> samplepoints;
	for (auto it : center_group)
	{
		Eigen::Vector3d p_c;
		p_c[0] = 1;
		p_c[1] = 0;
		p_c[2] = 0;
		double height = ui.height->value() / 1000;
		double num = high * it.x + it.y;
		if (cloud_ptr->points_[high*it.x + it.y].z() < height && !isnan(cloud_ptr->points_[high*it.x + it.y].z()))
		{
			double x = cor[0] + it.x / 2 - 3;
			double y = cor[1] + it.y / 2 - 3;
			samplepoints.push_back(it);
			//painter.drawEllipse(x, y, 6, 6);
			cloud_ptr->colors_[high*it.x + it.y] = p_c;
		}
	}
	painter.drawEllipse(pointxy.x, pointxy.y, 6, 6);
	cv::Point2d f_point = samplepoints.front();
	cv::Point2d b_point = samplepoints.back();
	int samNum = ui.sampleNum->value();
	double x_step = (b_point.x - f_point.x) / (samNum - 1);
	double y_step = (b_point.y - f_point.y) / (samNum - 1);
	samplepoints.clear();
	Path.clear();
	for (int i = 0; i < samNum; i++)
	{
		cv::Point2d sample;
		sample.x = i * x_step + f_point.x;
		sample.y = i * y_step + f_point.y;
		if (i == samNum - 1)
		{
			sample = b_point;
		}
		samplepoints.push_back(sample);
		double x = cor[0] + sample.x / 2 - 3;
		double y = cor[1] + sample.y / 2 - 3;
		painter.drawEllipse(x, y, 6, 6);
		Path.push_back(cloud_ptr->points_[high*sample.x + sample.y]);
	}
	ui.label->setPixmap(pixmap);
	cloud_ptr->RemoveNonFinitePoints();
	
	auto box=std::make_shared<open3d::geometry::OrientedBoundingBox>(); 
	box->CreateFromPoints(cloud_ptr->points_);
	box->GetOrientedBoundingBox();
	box->color_ = { 0,0,1 };
	open3d::visualization::DrawGeometries({cloud_ptr,box});
}
bool QtWidgetsimgrecognition::recognitionLeg()
{
	//updataimg();
	cv::Mat gray;
	cv::Mat img_copy;
	cv::Mat g_srcImage = save_image;
	cv::Mat g_dstImage = cv::Mat::zeros(g_srcImage.size(), g_srcImage.type());
	double compareValue = ui.comp->value() / 100;
	double lightValue = ui.light->value();
	g_srcImage.convertTo(g_dstImage, -1, compareValue, lightValue);
	img_copy = g_dstImage.clone();
	cv::cvtColor(img_copy, gray, cv::COLOR_BGR2GRAY);
	QImage img;
	if (ui.depthImg->isChecked())
	{
		
		img = QImage((const unsigned char*)(depth_image.data),
			g_dstImage.cols, g_dstImage.rows, QImage::Format_Grayscale8);
	}
	else
	{
		img = QImage((const unsigned char*)(gray.data),
			g_dstImage.cols, g_dstImage.rows, QImage::Format_Grayscale8);
	}//设定图像大小自适应label窗口的大小
	img = img.scaled(ui.label->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	int cor[4];
	std::vector<cv::Point2d> point_group;
	ui.label->getXY(cor);
	int wigth = (cor[2] - cor[0]) * 2;
	int high = (cor[3] - cor[1]) * 2;
	ui.x_first->setText(QString::fromStdString(std::to_string(cor[0])));
	ui.y_first->setText(QString::fromStdString(std::to_string(cor[1])));
	ui.x_final->setText(QString::fromStdString(std::to_string(cor[2])));
	ui.y_final->setText(QString::fromStdString(std::to_string(cor[3])));
	
	point_group = recognition2lines(depth_image, cor);
	
	if (point_group.size() < wigth * 2)
	{
		return false;
	}
	QPixmap pixmap = QPixmap::fromImage(img);
	QPainter painter(&pixmap);
	painter.setPen(QPen(Qt::green, 2));
	painter.setRenderHint(QPainter::Antialiasing, true);  //设置渲染,启动反锯齿
	save_2D_lins.clear();
	for (auto it : point_group)
	{
		double x = cor[0] + it.x / 2 - 1;
		double y = cor[1] + it.y / 2 - 1;
		painter.drawEllipse(x, y, 2, 2);
		//save_2D_lins.push_back(x);
		//save_2D_lins.push_back(y);
	}
	for (auto it : center_group)
	{
		double x = cor[0] + it.x / 2 ;
		double y = cor[1] + it.y / 2 ;
		painter.drawEllipse(x, y, 1, 1);
		save_2D_lins.push_back(x);
		save_2D_lins.push_back(y);
	}
	painter.drawEllipse(cor[0]+pointxy.x/2-5, cor[1]+pointxy.y/2-5, 10, 10);
	
	if (ui.cmpsample->isChecked())
	{
		for (int i = 0; i < read_2D_lins.size(); i += 2)
		{
			double x = read_2D_lins[i];
			double y = read_2D_lins[i+1];
			painter.setPen(QPen(Qt::red, 2));
			painter.drawEllipse(x, y, 1, 1);
		}
	}
	//半径为50的圆
	//painter.drawEllipse(0, 0, 100, 100);
	//read pointcloud//
	/*open3d::geometry::PointCloud source;
	source = save_cloud;
	std::vector<Eigen::Vector3d> group = cut3DMod(source);
	open3d::geometry::PointCloud cloud;
	auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
	cloud_ptr->points_ = group;
	cloud_ptr->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(200));
	//              //
	std::vector<Eigen::Vector3d> color;
	for (auto it : group)
	{
		Eigen::Vector3d p_c;
		p_c[0] = 0;
		p_c[1] = 1;
		p_c[2] = 0;
		color.push_back(p_c);
	}
	cloud_ptr->colors_ = color;

	
	cloud_ptr->RemoveNonFinitePoints();
	*/
	ui.label->setPixmap(pixmap);
	ui.MonitorIMG->setPixmap(pixmap);
	return true;
}


std::vector<cv::Point2d> QtWidgetsimgrecognition::CutImg(cv::Mat image,int F[4])
{
	cv::Rect rect(F[0]*2, F[1]*2, (F[2]-
		F[0])*2, (F[3]-F[1])*2);
	cv::Mat image_roi = image(rect);
	cv::Mat output;
	cv::Canny(image_roi, output, 100, 300);
	cv::Mat contoursInv;
	cv::threshold(output, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
	std::vector<cv::Vec4f> lines;
	std::vector<cv::Vec4f> lines_p;
	cv::HoughLinesP(output,lines,1, 3.14 / 180, 5,60,40);
	cv::Scalar color = cv::Scalar(0, 0, 0);
	cv::Vec2f k_b;
	k_b[0] = 0;
	k_b[1] = 0;
	for (auto it : lines)
	{
		double k = (it[3] - it[1]) / (it[2] - it[0]);
		double b = it[3] - k * it[2];
		if (abs(k) > 0.3)
		{
			continue;
		}
		if (!k_b[0])
		{
			k_b[0] = k;
			k_b[1] = b;
			cv::Vec4f point;
			point[0] = 0;
			point[1] = b;
			point[2] = rect.width;
			point[3] = k * point[2] + b;
			lines_p.push_back(point);
			continue;
		}
		if (abs(k - k_b[0]) < 0.05&&abs(b - k_b[1]) > 20)
		{
			cv::Vec4f point;
			point[0] = 0;
			point[1] = b;
			point[2] = rect.width;
			point[3] = k * point[2] + b;
			lines_p.push_back(point);
			break;
		}

	}
	for (size_t i = 0; i < lines_p.size(); i++) {
		cv::Vec4f temp = lines_p[i];
		line(image_roi, cv::Point(temp[0], temp[1]), cv::Point(temp[2], temp[3]), color, 2);
	}
	//imshow("houghLinesP img", image_roi);
	std::vector<cv::Point2d> p_group;
	for (auto it : lines_p)
	{
		double k = (it[3] - it[1]) / (it[2] - it[0]);
		double b = it[3] - k * it[2];
		for (int x = 0; x < rect.width; x++)
		{
			cv::Point2d point;
			point.x = int(x);
			point.y = int(k * x + b);
			p_group.push_back(point);
		}
	}
	center_group.clear();
	
	for (int x = 0; x < rect.width; x+=1)
	{
		int point_x=0;
		int point_y=0;
		double k=0;
		double b=0;
		for (auto it : lines_p)
		{
			 k += (it[3] - it[1]) / (it[2] - it[0]);
			 b += it[3] - (it[3] - it[1]) / (it[2] - it[0]) * it[2];
		}
		k = k / 2;
		b = b / 2;
		cv::Point2d point;
		point.x = int(x);
		point.y = int(k * x + b);
		center_group.push_back(point);
	}
	return p_group;
}
//rect既是要截取的区域
std::vector<cv::Point2d> QtWidgetsimgrecognition::recognition2lines(cv::Mat image, int F[4])
{
	updataimg();
	cv::Mat img_copy;
	image.copyTo(img_copy);
	if (F[2] == 0 && F[3] == 0)
	{
		F[0] = 3;
		F[1] = 192;
		F[2] = 135;
		F[3] = 393;
	}
	cv::Rect rect(F[0] * 2, F[1] * 2, (F[2] -
		F[0]) * 2, (F[3] - F[1]) * 2);
	
	cv::Mat image_roi = img_copy(rect);
	cv::Mat output;
	cv::Canny(image_roi, output, 100, 300);
	cv::Mat contoursInv;
	cv::threshold(output, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
	std::vector<cv::Vec4f> lines;
	std::vector<cv::Vec4f> lines_p;
	cv::HoughLinesP(output, lines, 1, 3.14 / 180, 5, 100, 40);
	cv::Scalar color = cv::Scalar(0, 0, 0);
	cv::Vec2f k_b;
	k_b[0] = 0;
	k_b[1] = 0;
	for (auto it : lines)
	{
		double k = (it[3] - it[1]) / (it[2] - it[0]);
		double b = it[3] - k * it[2];
		if (abs(k)<0.4&&abs(k)>1.1)
		{
			continue;
		}
		if (!k_b[0])
		{
			k_b[0] = k;
			k_b[1] = b;
			cv::Vec4f point;
			point[0] = 0;
			point[1] = b;
			point[2] = rect.width;
			point[3] = k * point[2] + b;
			lines_p.push_back(point);
			continue;
		}
		if (abs(k - k_b[0]) < 10&&(k*k_b[0])<0)
		{
			cv::Vec4f point;
			point[0] = 0;
			point[1] = b;
			point[2] = rect.width;
			point[3] = k * point[2] + b;
			lines_p.push_back(point);
			pointxy.x = (b - k_b[1]) / (k_b[0] - k);
			pointxy.y = k * (pointxy.x) + b;
			break;
		}
	}
	
	for (size_t i = 0; i < lines_p.size(); i++) {
		cv::Vec4f temp = lines_p[i];
		line(image_roi, cv::Point(temp[0], temp[1]), cv::Point(temp[2], temp[3]), color, 2);
	}
	//imshow("houghLinesP img", image_roi);
	std::vector<cv::Point2d> p_group;
	if (lines_p.size() < 2)
	{
		return p_group;
	}
	for (auto it : lines_p)
	{
		double k = (it[3] - it[1]) / (it[2] - it[0]);
		double b = it[3] - k * it[2];
		for (int x = 0; x < rect.width; x++)
		{
			cv::Point2d point;
			point.x = int(x);
			point.y = int(k * x + b);
			p_group.push_back(point);
		}
	}
	
	double k1;
	double k2;
	double k_center;
	k1 = (lines_p[0][3] - lines_p[0][1]) / (lines_p[0][2] - lines_p[0][0]);
	k2 = (lines_p[1][3] - lines_p[1][1]) / (lines_p[1][2] - lines_p[1][0]);
	double a_k = (2 - k1 * k1 - k2 * k2) / (k1 + k2);
	k_center = sqrt(4+ a_k * a_k) / 2 - a_k / 2;
	double o = atan(k1);
	double p = atan(k2);
	double k3 = tan((o + p) / 2);
	current_angle = (o + p) / 2;
	point2D_groups.clear();
	
	Eigen::Vector3d points3D;
	float error = 10;
	double k = k3;
	double b = 0;
	b = pointxy.y - k * pointxy.x;
	current_b = b;
	for (int i = -1; i < 1; i++)
	{
		center_group.clear();
		point2D_groups.clear();
		cv::Point2d point2D;
		
		for (int x = pointxy.x -30; x < pointxy.x + 65; x += 1)
		{
			cv::Point2d point;
			point.x = int(x);
			point.y = int(k * x + b) +i ;//ui.centerOffset->value()
			center_group.push_back(point);
			point2D.x = int(x) + +F[0] * 2;
			point2D.y = int(k * x + b) + F[1] * 2 +i ;
			point2D_groups.push_back(point2D);
		}
		cmperror();
		if (abs(Calerror) <= abs(error))
		{
			error = Calerror;
			imgOffset = i;
		}
	}
	//current_b += imgOffset;
	center_group.clear();
	point2D_groups.clear();
	for (int x = pointxy.x -30; x < pointxy.x + 65; x += 1)
	{
		double k = k3;
		double b = 0;
		cv::Point2d point2D;
		b = pointxy.y - k * pointxy.x;
		cv::Point2d point;
		point.x = int(x);
		point.y = int(k * x + b) + imgOffset;//ui.centerOffset->value()
		center_group.push_back(point);
		point2D.x = int(x) + +F[0] * 2;
		point2D.y = int(k * x + b) + F[1] * 2 + imgOffset;
		point2D_groups.push_back(point2D);
	}
	Calerror = error;
	ui.centerOffset->setValue(imgOffset);
	ui.errorNUM->setValue(Calerror);
	for (auto it : point2D_groups)
	{
		points3D += save_cloud.points_[1440 * it.y + it.x];
	}
	sample_point = points3D / (point2D_groups.size());
	string str;
	str += " x:";
	str += to_string(sample_point.x()*1000);
	str += " y:";
	str += to_string(sample_point.y()*1000);
	str += " z:";
	str += to_string(sample_point.z()*1000);
	ui.outputXYZ->setText(QString::fromStdString(str));
	return p_group;
}
void QtWidgetsimgrecognition::cannyMod()
{
	cv::Mat gray;
	cv::Mat img_copy;
	cv::Mat image = cv::imread(filename.toStdString());
	cv::Mat g_srcImage = image;
	cv::Mat g_dstImage = cv::Mat::zeros(g_srcImage.size(), g_srcImage.type());
	QImage img;
	if (filename.length() > 1)
	{
		double compareValue = ui.comp->value() / 100;
		double lightValue = ui.light->value();
		g_srcImage.convertTo(g_dstImage, -1, compareValue, lightValue);
		img_copy = g_dstImage.clone();
		cv::cvtColor(img_copy, gray, cv::COLOR_BGR2GRAY);
		img = QImage((const unsigned char*)(gray.data),
			g_dstImage.cols, g_dstImage.rows, QImage::Format_Grayscale8);
		//设定图像大小自适应label窗口的大小
		img = img.scaled(ui.label->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
		//
	}
	int cor[4];
	std::vector<cv::Point2d> p_group;
	ui.label->getXY(cor);
	p_group=CutImg(gray, cor);
	QPixmap pixmap = QPixmap::fromImage(img);
	QPainter painter(&pixmap);
	painter.setPen(QPen(Qt::green, 2));
	painter.setRenderHint(QPainter::Antialiasing, true);  //设置渲染,启动反锯齿
	for (auto it : p_group)
	{
		double x = cor[0]+it.x/2;
		double y = cor[1]+it.y/2;
		painter.drawEllipse(x,y,1,1);

	}//半径为50的圆
	//painter.drawEllipse(40, 40, 10, 10);
	ui.label->setPixmap(pixmap);
}
void QtWidgetsimgrecognition::cmperror()
{
	//cutCloud(save_cloud);
	Eigen::Vector3d point0;
	Eigen::Vector3d point1;
	Eigen::Vector3d point2;
	int size = point2D_groups.size();
	double error_0=0;
	double error_1=0;
	int num = 0;
	for (int i=0;i< point2D_groups.size();i++)
	{
		double y_a = cos(i / size * 3.14 / 2);
		double z_b = sin(i / size * 3.14 / 2);
		double high_value=0;
		double low_value = 0;
		int x0 = point2D_groups[i].x;
		int y0 = point2D_groups[i].y;
		point0 = save_cloud.points_[1440 * y0 + x0];
		if (isnan(point0.x()))
		{
			continue;
		}
		for (int j = 0; j < 15; j++)
		{
			int x1 = point2D_groups[i].x;
			int y1 = point2D_groups[i].y+j;
			point1 = save_cloud.points_[1440 * y1 + x1];
			int x2 = point2D_groups[i].x;
			int y2 = point2D_groups[i].y-j;
			point2 = save_cloud.points_[1440 * y2 + x2];
			if (isnan(point1.x()) || isnan(point2.x()))
			{
				continue;
			}
			if (abs(-point1.z() + point0.z())<0.002)
			{
				error_0 += (y_a * (point1.x() - point0.x()) + z_b*2 * (-point1.z() + point0.z()))*(10 - j);
				error_1 += (y_a * (point2.x() - point0.x()) + z_b*2 * (-point2.z() + point0.z()))*(10 - j);
			}
			num++;
		}
	}
	Calerror = (error_0 - error_1) / num * 1000;
	
}
std::vector<Eigen::Vector3d> QtWidgetsimgrecognition::cut3DMod(open3d::geometry::PointCloud source)
{
	int point[4];
	ui.label->getXY(point);
	Eigen::Vector3d point1;
	Eigen::Vector3d point2;
	//point1 = source.points_[1440 * point[1] * 2 + point[0] * 2];
	//point2 = source.points_[1440 * point[3] * 2 + point[2] * 2];
	std::vector<Eigen::Vector3d> group;
	auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
	auto cloud_ptr2 = std::make_shared<open3d::geometry::PointCloud>();
	for (int i = point[0] * 2; i < point[2] * 2; i++)
	{
		for (int j = point[1] * 2; j < point[3] * 2; j++)
		{
			point1 = source.points_[1440 * j  + i ];
			group.push_back(point1);
		}
	}
	source.points_ = group;
	cloud_ptr->points_ = group;
	cloud_ptr2=cloud_ptr->VoxelDownSample(0.00001);
	cloud_ptr2->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(500));
	
	std::vector<Eigen::Vector3d> color;
	for (int i=0;i<cloud_ptr2->points_.size();i++)
	{
		if (abs(cloud_ptr2->normals_[i].y()) <0.1&&abs(cloud_ptr2->normals_[i].x())<1)
		{
			Eigen::Vector3d p_c;
			p_c[0] = 1;
			p_c[1] = 0;
			p_c[2] = 0;
			color.push_back(p_c);
		}
		else
		{
			Eigen::Vector3d p_c;
			p_c[0] = 0;
			p_c[1] = 1;
			p_c[2] = 0;
			color.push_back(p_c);
		}
	}
	cloud_ptr2->colors_ = color;
	//open3d::visualization::DrawGeometries({cloud_ptr2});
	return group;
}
std::vector<Eigen::Vector3d> QtWidgetsimgrecognition::cutCloud()
{
	open3d::geometry::PointCloud source;
	source = save_cloud;
	Eigen::Vector3d point1;
	Eigen::Vector3d point2;
	Eigen::Vector3d point0;
	std::vector<Eigen::Vector3d> group;
	std::vector<Eigen::Vector3d> color;
	Eigen::Vector3d green;
	green[0] = 0;
	green[1] = 1;
	green[2] = 0;
	Eigen::Vector3d red;
	red[0] = 1;
	red[1] = 0;
	red[2] = 0;
	auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
	auto cloud_ptr2 = std::make_shared<open3d::geometry::PointCloud>();
	int size = save_2D_lins.size();
	for (int i = 0; i < size; i+=2)
	{
		double y_a = cos(i / size * 3.14 / 2);
		double z_b = sin(i / size * 3.14 / 2);
		int x0 = save_2D_lins[i]*2+2;
		int y0 = save_2D_lins[i+1]*2+2;
		point0 = source.points_[1440 * y0 + x0];
		if (isnan(point0.x()))
		{
			continue;
		}
		group.push_back(point0);
		color.push_back(red);
		for (int j = 0; j < 30; j++)
		{
			int x1 = save_2D_lins[i]*2+2;
			int y1 = save_2D_lins[i+1]*2 + j+2;
			point1 = source.points_[1440 * y1 + x1];
			int x2 = save_2D_lins[i]*2+2;
			int y2 = save_2D_lins[i+1]*2-j+2;
			point2 = source.points_[1440 * y2 + x2];
			if (isnan(point1.x()) || isnan(point2.x()))
			{
				continue;
			}
			group.push_back(point1);
			color.push_back(green);
			group.push_back(point2);
			color.push_back(green);
		}
	}
	source.points_ = group;
	cloud_ptr->points_ = group;
	cloud_ptr->colors_ = color;
	cloud_ptr = cloud_ptr->VoxelDownSample(0.00001);
	cloud_ptr->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(500));
	cloud_ptr->RemoveNonFinitePoints();
	open3d::visualization::DrawGeometries({cloud_ptr});
	return group;
}

void QtWidgetsimgrecognition::ShowCutPoint()
{
	open3d::geometry::PointCloud source, target;
	source = save_cloud;
	std::vector<Eigen::Vector3d> group=cut3DMod(source);
	open3d::geometry::PointCloud cloud;
	cloud.points_ = group;
	cloud.EstimateNormals();
}
bool QtWidgetsimgrecognition::camerainit()
{
	if (!CAMERACONNECTED)
	{
		// Initialize RVC X system.
		RVC::SystemInit();


		// Choose RVC X Camera type (USB or GigE).
		RVC::Device devices[10];

		size_t actual_size = 0;
		SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::GigE);

		// Find whether any Camera is connected or not.
		if (actual_size == 0) {
			std::cout << "Can not find any Camera!" << std::endl;
			return 0;
		}

		// Create a RVC X Camera.
		x1 = RVC::X1::Create(devices[0]);

		// Open RVC X Camera.
		std::string str = "camera is connnected";
		ui.cameratype->setText(QString::fromStdString(str));
		ui.cameratype->setStyleSheet(QString::fromStdString("color:green"));
		CAMERACONNECTED = true;
	}
	return CAMERACONNECTED;
}
void QtWidgetsimgrecognition::cameraOpen()
{
	x1.Open();

		// Test RVC X Camera is opened or not.
		if (!x1.IsOpen()) {
			std::cout << "RVC X Camera is not opened!" << std::endl;
			RVC::X1::Destroy(x1);
			RVC::SystemShutdown();
			return ;
		}
	// Set capture parameters
	RVC::X1::CaptureOptions cap_opt;
	// Transform point map's coordinate to left/right(RVC::CameraID_Left/RVC::CameraID_Right) camera or reference
	// plane(RVC::CameraID_NONE)
	if (ui.chooseCamera->currentIndex())
	{
		cap_opt.transform_to_camera = RVC::CameraID_Right;
	}
	else
	{
		cap_opt.transform_to_camera = RVC::CameraID_Left;
	}
	//Left==1;RIGHT=2
	// Set camera exposure time (3~100) ms
	cap_opt.exposure_time_2d = ui.light2d->value();
	cap_opt.exposure_time_3d = ui.light3d->value();
	// range in [0, 10], default = 5. The contrast of point less than this value will be treat * as invalid point and
	// be removed.
	//cap_opt.light_contrast_threshold = ui.sampleValue->value();
	// edge control after point matching, range in [0, 10], default = 2. The big the value, the more edge * noise to be
	// removed.
	cap_opt.filter_range = ui.edgeNoise->value();
	
	const std::string save_directory = "./Data/";
	MakeDirectories(save_directory);
	// Capture a point map and a image with default setting.
	if (x1.Capture(cap_opt) == true) {
		// Get point map data.
		RVC::PointMap pm = x1.GetPointMap();
		std::string pm_addr = save_directory + "test1.ply";
		std::cout << "save point map to file: " << pm_addr << std::endl;
		save_cloud = getOpen3Dmap(pm);
		//SavePlyFile(pm_addr.c_str(), pm);
		// Get image data. choose left or right side. the point map is map to left image.
		RVC::Image img = x1.GetImage();
		std::string img_addr = save_directory + "test1.png";
		std::cout << "save image to file: " << img_addr << std::endl;
		save_image = TranMatImage(img);
		//SaveMatImage(img_addr.c_str(), img);
		 // Get depth image.
		depth_image = ExtractDepthImage(save_cloud,save_image,ui.height->value());
		std::string depth_addr = save_directory + "depth.tiff";
		std::cout << "save depth image to file: " << depth_addr << std::endl;
	}
	else {
		std::cout << "RVC X Camera capture failed!" << std::endl;
	}
	updataimg();
	//x1.Close();
	// Close RVC X Camera.
}
void QtWidgetsimgrecognition::cameraclose()
{
	x1.Close();
	// Destroy RVC X Camera.
	RVC::X1::Destroy(x1);

	// Shutdown RVC X System.
	RVC::SystemShutdown();
	CAMERACONNECTED = false;
	std::string str = "camera is disconnnected";
	ui.cameratype->setText(QString::fromStdString(str));
	ui.cameratype->setStyleSheet(QString::fromStdString("color:red"));
}
open3d::geometry::PointCloud QtWidgetsimgrecognition::getOpen3Dmap(RVC::PointMap &pm)
{
	open3d::geometry::PointCloud source;
	std::vector<Eigen::Vector3d> group;
	const unsigned int pm_sz = pm.GetSize().height *  pm.GetSize().width;
	const double *pm_data = pm.GetPointDataPtr();
	for (int i = 0; i < pm_sz; i++, pm_data += 3) {
		Eigen::Vector3d point;
		point[0] = pm_data[0];
		point[1] = pm_data[1];
		point[2] = pm_data[2];
		group.push_back(point);
	}
	source.points_ = group;
	return source;
}
void QtWidgetsimgrecognition::readRobotPose()
{
	int input;
	int num = ui.dataNum->value();
	uint16_t status[5];
	uint16_t port = ui.port->value();
	QString str = ui.IPaddr->text();
	TCP con(str.toStdString(),port,ui.slaveID->value());
	if (con.connect()) {
		if (con.read_holding_registers(ui.addr->value(), 5, status)) {
			for (int i = 0; i < 1; i++) {
				fprintf(stdout, "%x\r\n", status[i]);
			}
		}
		string output = "output";
		for (int i = 0; i < 5; i++)
		{

			output += ":";
			output += std::to_string(status[i]);
		}
		ui.code->setText(QString::fromStdString(output));
		string str1;
		string str2;
		//con.write_holding_register(10, 100);
		str1 = ItoB(status[1]);
		str2 = ItoB(status[0]);
		double num = BtoD(str1+str2);
		//
		double num1= BtoD(to_IEEE754(1));
		//
		
		int output1[2];
		 
	//	BtoReal(to_IEEE754(4),output1);
		ui.TCPTXET->setText(QString::fromStdString(std::to_string(num)));

		con.close();
	 }
}
void QtWidgetsimgrecognition::Sampleinit()
{
    QString file_name = "eyetohand.txt";
	ifstream inf(file_name.toStdString(), ios_base::in);
	ui.eyetohandPath->setText(file_name);
	if (inf.is_open())
	{
		string line;
		int count = 0;
		int index = 0;
		float point[3];
		vector<float> point_group;
		while (!inf.eof())
		{
			inf >> line;
			point_group.push_back(atof(line.c_str()));
		}
		if (point_group.size() < 16)
		{
			return;
		}
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				eyetohandMat(i, j) = point_group[i * 4 + j];
			}
		}
		inf.close();
	}
	readSampleRobot();
	readErrorPoint();
}
void  QtWidgetsimgrecognition::readEyetohand()
{
	char buf[1024] = { '\0' };
	QString file_name = QFileDialog::getOpenFileName(this,
		tr("open image"),
		".",
		tr("Image file(*.txt)"));
	ui.eyetohandPath->setText(file_name);
	ifstream inf(file_name.toStdString(), ios_base::in);
	if (inf.is_open())
	{
		string line;
		int count = 0;
		int index = 0;
		float point[3];
		vector<float> point_group;
		while (!inf.eof())
		{
			inf >> line;
			point_group.push_back(atof(line.c_str()));
		}
		if (point_group.size() < 16)
		{
			return;
		}
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				eyetohandMat(i, j) = point_group[i * 4 + j];
			}
		}
		inf.close();
	}
}
void  QtWidgetsimgrecognition::readSampleRobot()
{
	char buf[1024] = { '\0' };
	Eigen::Matrix4f Samples;
	QString file_name = "SampleCameraPath.txt";
	ifstream inf(file_name.toStdString(), ios_base::in);
	if (inf.is_open())
	{
		string line;
		int count = 0;
		int index = 0;
		float point[3];
		vector<float> point_group;
		while (!inf.eof())
		{
			inf >> line;
			point_group.push_back(atof(line.c_str()));
		}
		if (point_group.size() < 30)
		{
			return;
		}
		for (int i = 0; i < 5; i++)
		{
			float pose[6];
			for (int j = 0; j < 6; j++)
			{
				
				pose[j] = point_group[i * 6 + j];
				SampleCameraPath[j][i]= point_group[i * 6 + j];
			}
			
		}
		inf.close();
	}
}
void  QtWidgetsimgrecognition::readSamplePath()
{
	char buf[1024] = { '\0' };
	Eigen::Matrix4f Samples;
	QString file_name = "SampleMotionPath.txt";
	ifstream inf(file_name.toStdString(), ios_base::in);
	if (inf.is_open())
	{
		string line;
		int count = 0;
		int index = 0;
		float point[3];
		vector<float> point_group;
		while (!inf.eof())
		{
			inf >> line;
			point_group.push_back(atof(line.c_str()));
		}
		if (point_group.size() != 150)
		{
			return;
		}
		for (int i = 0; i < 50; i++)
		{
			float pose[6];
			for (int j = 0; j < 3; j++)
			{
				pose[j] = point_group[i * 3 + j];
				SampleCameraPath[j][i] = point_group[i * 5 + j];
			}

		}
		inf.close();
	}
}
void  QtWidgetsimgrecognition::readErrorPoint()
{
	char buf[1024] = { '\0' };
	Eigen::Matrix4f Samples;
	QString file_name = "SampleErrorPoint.txt";
	ifstream inf(file_name.toStdString(), ios_base::in);
	if (inf.is_open())
	{
		string line;
		int count = 0;
		int index = 0;
		float point[3];
		vector<float> point_group;
		while (!inf.eof())
		{
			inf >> line;
			point_group.push_back(atof(line.c_str()));
		}
		if (point_group.size() <15)
		{
			return;
		}
		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 5; j++)
			{
				SampleFivePoints[j][i] = point_group[i * 5 + j];
			}
		}
		inf.close();
	}
}
Eigen::Matrix4f QtWidgetsimgrecognition::Euler2Mat(float pose[6], RotationStatus direction)
{
	float x, y, z;
	x = pose[3]/180*3.14;
	y = pose[4]/180*3.14;
	z = pose[5]/180*3.14;
	Eigen::Matrix4f R_z;
	Eigen::Matrix4f R_y;
	Eigen::Matrix4f R_x;
	Eigen::Matrix4f R_T;
	R_z << cos(z), -sin(z), 0, 0, sin(z), cos(z), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
	R_y << cos(y), 0, sin(y), 0, 0, 1, 0, 0, -sin(y), 0, cos(y), 0, 0, 0, 0, 1;
	R_x << 1, 0, 0, 0, 0, cos(x), -sin(x), 0, 0, sin(x), cos(x), 0, 0, 0, 0, 1;
	switch (direction)
	{
	case(RXYZn):
		R_T = R_x * R_y*R_z;
		break;
	case(RXZYn):
		R_T = R_x * R_z*R_y;
		break;
	case(RYXZn):
		R_T = R_y * R_x*R_z;
		break;
	case(RYZXn):
		R_T = R_y * R_z*R_x;
		break;
	case(RZXYn):
		R_T = R_z * R_x*R_y;
		break;
	case(RZYXn):
		R_T = R_z * R_y*R_x;
		break;
	}
	R_T(0, 3) = pose[0];
	R_T(1, 3) = pose[1];
	R_T(2, 3) = pose[2];
	return R_T;
}

void saveCameraSample(double arr[6][5])
{
	ofstream out("SampleCameraPath.txt");
	string str;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 6;j++)
		{
			str += to_string(arr[j][i]);
			str += " ";
		}
		str += "\n";
	}
	out << str << endl;
}
void saveErrorPoint(double arr[5][5])
{
	ofstream out("SampleErrorPoint.txt");
	string str;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			str += to_string(arr[j][i]);
			str += " ";
		}
		str += "\n";
	}
	out << str << endl;
}
void saveMotionSample(double arr[3][50])
{
    ofstream out("SampleMotionPath.txt");
	string str;
	for (int i = 0; i < 50; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			str += to_string(arr[j][i]);
			str += " ";
		}
		str += "\n";
	}
	out << str << endl;
}
void QtWidgetsimgrecognition::saveSample()
{
	saveCameraSample(SampleCameraPath);
	saveErrorPoint(SampleFivePoints);
}
void QtWidgetsimgrecognition::save_2D_lines()
{
	ofstream out("Sample2DLine.txt");
	std::string str;
	for (int i = 0; i < save_2D_lins.size(); i+=2)
	{
		
			str += to_string(save_2D_lins[i]);
			str += " ";
			str += to_string(save_2D_lins[i+1]);
		str += "\n";
	}
	out << str << endl;
}
void QtWidgetsimgrecognition::read_2D_lines()
{
	char buf[1024] = { '\0' };
	Eigen::Matrix4f Samples;
	QString file_name = "Sample2DLine.txt";
	ifstream inf(file_name.toStdString(), ios_base::in);
	if (inf.is_open())
	{
		string line;
		int count = 0;
		int index = 0;
		float point[3];
		vector<float> point_group;
		read_2D_lins.clear();
		while (!inf.eof())
		{
			inf >> line;
			read_2D_lins.push_back(atof(line.c_str()));
		}
		inf.close();
	}
}
void QtWidgetsimgrecognition::saveTarget()
{
	recognitionTarget();
	int index = ui.imgSamplePoint->currentIndex();
	for (int i = 0; i < 10; i++)
	{
		SamplePath[0][index * 10 + i] = Path[i].x();
		SamplePath[1][index * 10 + i] = Path[i].y();
		SamplePath[2][index * 10 + i] = Path[i].z();
	}
	//
	int input;
	int num = ui.dataNum->value();
	uint16_t status[12];
	uint16_t port = ui.port->value();
	QString str = ui.IPaddr->text();
	TCP con(str.toStdString(), port, ui.slaveID->value());
	if (con.connect()) {
		if (con.read_holding_registers(30, 12, status)) {
			for (int i = 0; i < 1; i++) {
				fprintf(stdout, "%x\r\n", status[i]);
			}
		}
		string output = "output";
		for (int i = 0; i < 6; i++)
		{
			string str1;
			string str2;
			str1 = ItoB(status[2*i+1]);
			str2 = ItoB(status[2*i]);
			double num = BtoD(str1 + str2);
			SampleCameraPath[i][index] = num;
			output += ":";
			output += std::to_string(num);
		}
		ui.TCPTXET->setText(QString::fromStdString(output));
		con.close();
	}
	
}
void QtWidgetsimgrecognition::saveOnePoint()
{
	bool is_ok=false;
	
	if (!recognitionLeg())
	{

		return;
	}
	int index = ui.imgSamplePoint->currentIndex();
	SampleFivePoints[0][index] = sample_point.x();
	SampleFivePoints[1][index] = sample_point.y();
	SampleFivePoints[2][index] = sample_point.z();
	SampleFivePoints[3][index] = current_angle;
	SampleFivePoints[4][index] = current_b;
	//
	int input;
	int num = ui.dataNum->value();
	uint16_t status[12];
	uint16_t port = ui.port->value();
	QString str = ui.IPaddr->text();
	TCP con1(str.toStdString(), port, ui.slaveID->value());
	if (con1.connect()) {
		if (con1.write_holding_register(60, 2))
		{
		}
		else
		{
			return;
		}
	}
	con1.close();
	TCP con(str.toStdString(), port, ui.slaveID->value());
	if (con.connect()) {
		if (con.read_holding_registers(30, 12, status)) {
			for (int i = 0; i < 1; i++) {
				fprintf(stdout, "%x\r\n", status[i]);
			}
		}
		string output = "output";
		for (int i = 0; i < 6; i++)
		{
			string str1;
			string str2;
			str1 = ItoB(status[2 * i + 1]);
			str2 = ItoB(status[2 * i]);
			double num = BtoD(str1 + str2);
			SampleCameraPath[i][index] = num;
			output += ":";
			output += std::to_string(num);
		}
		ui.TCPTXET->setText(QString::fromStdString(output));
		con.close();
	}
	TCP con2(str.toStdString(), port, ui.slaveID->value());
	if (con2.connect()) {
		if (con2.write_holding_register(60, 0))
		{
		}
		else
		{
			return;
		}
	}
	con1.close();

}
void QtWidgetsimgrecognition::sendOffset(float offset[3][10])
{
	int input;
	int num = ui.dataNum->value();
	uint16_t status[60];
	uint16_t port = ui.port->value();
	QString str = ui.IPaddr->text();
	TCP con(str.toStdString(), port, ui.slaveID->value());
	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int num[2];
			BtoReal(to_IEEE754(offset[j][i]), num);
			status[(j + i * 3) * 2] = num[1];
			status[(j + i * 3) * 2+1] = num[0];
		}
	}
	con.write_holding_registers(300, 60,status);
	con.close();
	TCP con1(str.toStdString(), port, ui.slaveID->value());
	con1.write_holding_register(6, 1);
	con1.close();
	
}
void QtWidgetsimgrecognition::sendPointOffset()
{
	int input;
	int num = ui.dataNum->value();
	uint16_t status[40];
	uint16_t port = ui.port->value();
	QString str = ui.IPaddr->text();
	

	for (int j = 0; j < 5; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			int num[2];
			if (abs(OffsetFivePoints[i][j]) > 0.01)
			{
				BtoReal(to_IEEE754(OffsetFivePoints[i][j]), num);
				status[8 * j + 2 * i] = num[1];
				status[8 * j + 2 * i + 1] = num[0];
			}
			else
			{
				status[8 * j + 2 * i] = 0;
				status[8 * j + 2 * i + 1] = 0;
			}
		}
	}
	TCP con(str.toStdString(), port, ui.slaveID->value());
	if (con.connect()) {
		con.write_holding_registers(300, 40, status);
	}
	else
	{
		return;
	}
	con.close();
	TCP con1(str.toStdString(), port, ui.slaveID->value());
	if (con1.connect()) {
		con1.write_holding_register(60, 0);
	}
	else
	{
		return;
	}
	con1.close();

}
void QtWidgetsimgrecognition::scanPLC()
{
	int input;
	int num = ui.dataNum->value();
	uint16_t status[2];
	uint16_t port = ui.port->value();
	QString str = ui.IPaddr->text();
	TCP con(str.toStdString(), port, ui.slaveID->value());
	if (con.connect()) {
		if (con.read_holding_registers(60, 3, status)) 
		{
		}
		else
		{
			return;
		}
   	}
	
	int index = 0;
	if (status[2] == 16256)
	{
		index = 1;
	}
	if (status[2] == 16384)
	{
		index = 2;
	}
	if (status[2] == 16448)
	{
		index = 3;
	}
	if (status[2] == 16512)
	{
		index = 4;
	}
	con.close();
	if (status[0] == 1)
	{
	   float offset[3];
	   camerainit();
	   cameraOpen();
	   
	   if (!recognitionLeg())
	   {
		   ui.recoResult->setText(QString::fromStdString("recognition failed"));
		   return;
	   }
	   else
	   {
		   ui.recoResult->setText(QString::fromStdString("recognition succeed"));
	   }
	   TCP con2(str.toStdString(), port, ui.slaveID->value());
	   if (con2.connect()) {
		   if (con2.write_holding_register(60, 0))
		   {
		   }
		   else
		   {
			   return;
		   }
	   }
	   TCP con3(str.toStdString(), port, ui.slaveID->value());
	   if (con3.connect()) {
		   int num[2];
		   uint16_t num2[4];
		   BtoReal(to_IEEE754(1), num);
		   num2[0] = num[1];
		   num2[1] = num[0];
		   num2[3] = 0;
		   num2[2] = 1;
		   if (con3.write_holding_registers(20, 4,num2))
		   {
		   }
		   else
		   {
			   return;
		   }
	   }
	   con3.close();
	   TCP con4(str.toStdString(), port, ui.slaveID->value());
	   if (con4.connect()) {
		   uint16_t num2[2];
		   num2[0] = 0;
		   num2[1] = 0;
		   if (con4.write_holding_registers(22, 2, num2))
		   {
		   }
		   else
		   {
			   return;
		   }
	   }
	   con4.close();
	   Eigen::Matrix4f mat;
	   Eigen::Matrix3f rot;
	   Eigen::Matrix3f eyetohandrot;
	  
	   float num[6];
	   for (int i = 0; i < 6; i++)
	   {
		   num[i] = SampleCameraPath[i][index];
	   }
	   mat = Euler2Mat(num, RXYZn);
	   rot=mat.topLeftCorner(3, 3);
	   Eigen::Vector3f delta;
	   Eigen::Vector3f deltaTCP; 
	   Eigen::Vector3f base_error;
	   //double effsetY= - SampleFivePoints[1][index] + sample_point.y();
	   double effsetY = -(-SampleFivePoints[4][index] + current_b)*0.21;
	   double offser_Ry = -SampleFivePoints[3][index] + current_angle;
	   delta(0) = 0;
	   delta(1) = effsetY;
	   delta(2) = 0;
	   eyetohandrot=eyetohandMat.topLeftCorner(3, 3);
	   base_error = eyetohandrot*delta;
	   deltaTCP = rot.inverse()*eyetohandrot*delta;
	   offset[0] = deltaTCP(0);
	   offset[1] = deltaTCP(1);
	   offset[2] = deltaTCP(2);
	   string str;
	   str += " x:";
	   str += to_string(offset[0] );
	   str += " y:";
	   str += to_string(offset[1]);
	   str += " z:";
	   str += to_string(offset[2]);
	   ui.errorXYZ->setText(QString::fromStdString(str));
	   OffsetFivePoints[0][index] = offset[0];
	   OffsetFivePoints[1][index] = offset[1];
	   OffsetFivePoints[2][index] = offset[2];
	   OffsetFivePoints[3][index] = offser_Ry;
	   if (index == 0)
	   {
		   ui.errorx_0->setValue(offset[0]);
		   ui.errory_0->setValue(offset[1]);
		   ui.errorz_0->setValue(offset[2]);
		   ui.RZ0->setValue(offser_Ry);
	   }
	   if (index == 1)
	   {
		   ui.errorx_1->setValue(offset[0]);
		   ui.errory_1->setValue(offset[1]);
		   ui.errorz_1->setValue(offset[2]);
		   ui.RZ1->setValue(offser_Ry);
	   }
	   if (index == 2)
	   {
		   ui.errorx_2->setValue(offset[0]);
		   ui.errory_2->setValue(offset[1]);
		   ui.errorz_2->setValue(offset[2]);
		   ui.RZ2->setValue(offser_Ry);
	   }
	   if (index == 3)
	   {
		   ui.errorx_3->setValue(offset[0]);
		   ui.errory_3->setValue(offset[1]);
		   ui.errorz_3->setValue(offset[2]);
		   ui.RZ3->setValue(offser_Ry);
	   }
	   if (index == 4)
	   {
		   ui.errorx_4->setValue(offset[0]);
		   ui.errory_4->setValue(offset[1]);
		   ui.errorz_4->setValue(offset[2]);
		   ui.RZ4->setValue(offser_Ry);
		   sendPointOffset();
		   int num;
		   num = ui.NumOfObjects->value();
		   num++;
		   ui.NumOfObjects->setValue(num);
	   }
      cameraclose();
	}
	
}

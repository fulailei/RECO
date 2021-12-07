#pragma once
#define    WIN32_LEAN_AND_MEAN
#include <opencv2/opencv.hpp>
#include <QtWidgets/QMainWindow>
#include "ui_QtWidgetsimgrecognition.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <QFileDialog>
#include <QApplication>
#include <iostream>
#include <memory>
#include <thread>
#include <Open3D/Open3D.h>
#include <RVC/RVC.h>
#include "FileIO.h"
#include "SaveImage.h"
#include "SavePointMap.h"
#include "TCP.h"
typedef enum rotation_status
{
	RXYZn, RXZYn, RYXZn, RYZXn, RZXYn, RZYXn	   //zyx
}RotationStatus;
struct CameraPoint
{
	cv::Point2d point2d;
	Eigen::Vector3d point3d;
};


class QtWidgetsimgrecognition : public QMainWindow
{
    Q_OBJECT

public:
    QtWidgetsimgrecognition(QWidget *parent = Q_NULLPTR);
	Eigen::Matrix4f Euler2Mat(float pose[6], RotationStatus direction);
	
private Q_SLOTS:
	void readimg();
	void SettingPage();
	void MonitorPage();
	void ScanOpen();
	void ScanClose();
	void showimg();
	void showgray();
	void showdepth();
	void updataimg();
	void readcloud();
	void recognitionTarget();
	bool recognitionLeg();
	std::vector<cv::Point2d> CutImg(cv::Mat image, int F[4]);
	std::vector<cv::Point2d> recognition2lines(cv::Mat image, int F[4]);
	void cannyMod();
	void cmperror();
	std::vector<Eigen::Vector3d> cut3DMod(open3d::geometry::PointCloud cloud);
	std::vector<Eigen::Vector3d> cutCloud();
	void ShowCutPoint();
	bool camerainit();
	void cameraOpen();
	void cameraclose();
	open3d::geometry::PointCloud getOpen3Dmap(RVC::PointMap & pm);
	void readRobotPose();
	void Sampleinit();
	void readEyetohand();
	void readSampleRobot();
	void readSamplePath();
	void readErrorPoint();
	void saveSample();
	void save_2D_lines();
	void read_2D_lines();
	void saveTarget();
	void saveOnePoint();
	void sendOffset(float offset[3][10]);
	void sendPointOffset();
	
	void scanPLC();
private:
	bool CAMERACONNECTED=false;
	Eigen::Matrix4f eyetohandMat;
	QString filename;
	std::vector<Eigen::Vector3d> Path;
	//std::vector<std::vector<Eigen::Vector3d>> SamplePath;
	double SamplePath[3][50] = { 0 };
	double SampleCameraPath[6][5] = { 0 };
	double SampleFivePoints[5][5] = { 0 };
	double OffsetFivePoints[4][5] = { 0 };
	std::vector<cv::Point2d> center_group;
	open3d::geometry::PointCloud save_cloud;
	cv::Mat save_image;
    Ui::QtWidgetsimgrecognitionClass ui;
	RVC::X1 x1;
	QTimer *timer;
	cv::Mat depth_image;
	Eigen::Vector3d offset_point;
	Eigen::Vector3d sample_point;
	cv::Point2d  pointxy;
	int imgOffset=0;
	float Calerror = 0;
	
	std::vector<double> save_2D_lins;
	std::vector<double> fix_2D_lins;
	std::vector<double> read_2D_lins;
	vector<cv::Point2d> point2D_groups;
	double current_angle;
	double current_b;
};

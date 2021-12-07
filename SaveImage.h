
#pragma once
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <RVC/RVC.h>

extern int SaveMatImage(const char *save_address, RVC::Image &img);
cv::Mat TranMatImage(RVC::Image &img);
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <iostream>
#include <filesystem>
#include<cmath>

#include "csv_util.h"

using namespace cv;
using namespace std;


void baselineMatchingfeatures(char* img_name, vector<float>& feature);
int SSD(char* img_name, vector<float>& t_features, vector<float>& d_features, vector<tuple<string,float>> &result);
int hg_intersection(char* img_name, vector<float>& t_features, vector<float>& d_features, vector<tuple<string, float>>& result);
int histogram(char* img_name, vector<float> &feature);
int multiHistogram(char* img_name, vector<float> &feature);
int TextureColor_histogram(char* img_name, vector<float> &feature);
int TextureColor_metric(char* img_name, vector<float> &T_colorhist, vector<float> &T_texthist,
                                        vector<float> &D_colorhist, vector<float> &D_texthist,
                                        vector<tuple<string, float>> &result);
int HSV_maskHG(char* img_name, vector<float>& feature);
int TextureColorHSV_metric(char* img_name, vector<float>& T_colorhist, vector<float>& T_texthist, vector<float>& T_hsvhist,
                                           vector<float>& D_colorhist, vector<float>& D_texthist, vector<float>& D_hsvhist,
                                           vector<tuple<string, float>>& result);

int sobelX3x3(Mat& src, Mat& dst);
int sobelY3x3(Mat& src, Mat& dst);
int magnitude(Mat& sx, Mat& sy, Mat& dst);
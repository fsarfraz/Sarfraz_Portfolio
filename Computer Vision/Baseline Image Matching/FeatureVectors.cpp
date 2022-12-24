#include "Header.h"

void baselineMatchingfeatures(char *img_name, vector<float> &feature){
    Mat img = imread(img_name, IMREAD_COLOR);
    for (int j = img.cols / 2 - 4; j < img.cols / 2 + 5; j++) {
        for (int k = img.rows / 2 - 4; k < img.rows / 2 + 5; k++) {
            for (int c = 0; c < 3; c++) {
                feature.push_back(img.at<Vec3b>(k, j)[c]);
            }
        }
    }
}
int histogram(char *img_name, vector<float> &feature) {
    Mat img = imread(img_name, IMREAD_COLOR);

    const int bins = 8;
    const int Hsize = 32;
    int dim[3] = { Hsize,Hsize,Hsize };
    Mat hist3d = Mat::zeros(3, dim, CV_32S);
    
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            int b = (img.at<Vec3b>(i, j)[0])/Hsize;
            int g = (img.at<Vec3b>(i, j)[1])/Hsize;
            int r = (img.at<Vec3b>(i, j)[2])/Hsize;
            hist3d.at<int>(b, g, r) += 1;

        }
    }
   for (int x = 0; x < bins; x++) {
        for (int y = 0; y < bins; y++) {
            for (int z = 0; z < bins; z++) {
                float sum = img.cols * img.rows;
                float hist = hist3d.at<int>(x, y, z);
                float normalized_val = hist / sum;

                feature.push_back(normalized_val);
            }
        }
    }
    return(0);
}

int multiHistogram(char* img_name, vector<float> &feature) {
    Mat img = imread(img_name, IMREAD_COLOR);

    const int bins = 8;
    const int Hsize = 32;
    int dim[3] = { Hsize,Hsize,Hsize };
    Mat hist3d_top = Mat::zeros(3, dim, CV_32S);
    Mat hist3d_bot = Mat::zeros(3, dim, CV_32S);
    
    //Top histogram
    for (int i = 0; i < img.rows/2; i++) {
        for (int j = 0; j < img.cols; j++) {
            int b = (img.at<Vec3b>(i, j)[0]) / Hsize;
            int g = (img.at<Vec3b>(i, j)[1]) / Hsize;
            int r = (img.at<Vec3b>(i, j)[2]) / Hsize;
            hist3d_top.at<int>(b, g, r) += 1;

        }
    }
    //Bottom histogram
    for (int i = img.rows / 2; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            int b = (img.at<Vec3b>(i, j)[0]) / Hsize;
            int g = (img.at<Vec3b>(i, j)[1]) / Hsize;
            int r = (img.at<Vec3b>(i, j)[2]) / Hsize;
            hist3d_bot.at<int>(b, g, r) += 1;

        }
    }
    //Top histogram nomalization
    for (int x = 0; x < bins; x++) {
        for (int y = 0; y < bins; y++) {
            for (int z = 0; z < bins; z++) {
                float sum = (img.cols * img.rows)/2;
                float hist_top = hist3d_top.at<int>(x, y, z);
                float normalized_val = hist_top / sum;

                feature.push_back(normalized_val);
            }
        }
    }
    //Bottom histogram normalization
    for (int x = 0; x < bins; x++) {
        for (int y = 0; y < bins; y++) {
            for (int z = 0; z < bins; z++) {
                float sum = (img.cols * img.rows)/2;
                float hist_bot = hist3d_bot.at<int>(x, y, z);
                float normalized_val = hist_bot / sum;

                feature.push_back(normalized_val);
            }
        }
    }
    return(0);

}

int TextureColor_histogram(char* img_name, vector<float> &feature) {
    //intializing magnitude filter
    Mat img = imread(img_name, IMREAD_COLOR);
    Mat temp, temp2, dst, gray;
    sobelX3x3(img, temp);
    sobelY3x3(img, temp2);
    magnitude(temp, temp2, dst); 
    cvtColor(dst, gray, COLOR_BGR2GRAY); // turning to grayscale
   
    //Histogram 1-d
    const int bins = 8;
    const int Hsize = 32;
    int dim[1] = {Hsize};
    Mat hist1D = Mat::zeros(1, dim, CV_32S);
    
    //texture histogram
    for (int i = 0; i < gray.rows; i++) {
        for (int j = 0; j < gray.cols; j++) {
            int gray_img = gray.at<unsigned char>(i, j) / Hsize;
            hist1D.at<int>(gray_img) += 1;
        }
    }

    //Texture histogram normalization
    for (int i = 0; i < bins; i++) {
        float sum = img.cols * img.rows;
        float hist_texture = hist1D.at<int>(i);
        float normalized_val = hist_texture / sum;

        feature.push_back(normalized_val);
    }
   return(0);
   
}

int HSV_maskHG(char* img_name, vector<float> &feature) {
    Mat img = imread(img_name, IMREAD_COLOR);
    Mat HSV,mask,res;
    cvtColor(img, HSV, COLOR_BGR2HSV);
    inRange(HSV, (100, 50, 50), (70, 255, 255), mask);
    bitwise_and(HSV, HSV, res, mask = mask);


    const int h_bins = 50;
    const int s_bins = 32;
    const int v_bins = 10;
    const int Hsize = h_bins * s_bins * v_bins;

    int dim[3] = { h_bins,s_bins,v_bins };
    Mat hist3d = Mat::zeros(3, dim, CV_32S);

    for (int i = 0; i < HSV.rows; i++) {
        for (int j = 0; j < HSV.cols; j++) {
            int h = (res.at<Vec3b>(i, j)[0]) / Hsize;
            int s = (res.at<Vec3b>(i, j)[1]) / Hsize;
            int v = (res.at<Vec3b>(i, j)[2]) / Hsize;
            hist3d.at<int>(h, s, v) += 1;

        }
    }
    for (int x = 0; x < h_bins; x++) {
        for (int y = 0; y < s_bins; y++) {
            for (int z = 0; z < v_bins; z++) {
                float sum = res.cols * res.rows;
                float hist = hist3d.at<int>(x, y, z);
                float normalized_val = hist / sum;

                feature.push_back(normalized_val);
            }
        }
    }

    return (0);
}





///////////////////PREVIOUS PROJECT FILTERS///////////////////
int sobelX3x3(Mat& src, Mat& dst){
    Mat temp;
    temp.create(src.size(), CV_16SC3);
    dst.create(src.size(), CV_16SC3);

    //Vec3b first_result;
    Vec3i result = { 0,0,0 };
    Vec3i result2 = { 0,0,0 };
    for (int i = 1; i < src.rows - 1; i++) {
        for (int j = 1; j < src.cols - 1; j++) {
            for (int c = 0; c < 3; c++) {
                result[c] = src.at<Vec3b>(i - 1, j - 1)[c] * -1 + src.at<Vec3b>(i - 1, j)[c] * 0 + src.at<Vec3b>(i - 1, j + 1)[c];
                //result[c] /= 10;
                temp.at<Vec3s>(i, j)[c] = result[c];
            }

        }
    }

    for (int j = 1; j < temp.cols - 1; j++) {
        for (int i = 1; i < temp.rows - 1; i++) {
            for (int c = 0; c < 3; c++) {
                result2[c] = temp.at<Vec3s>(i - 1, j - 1)[c] + temp.at<Vec3s>(i, j - 1)[c] * 2 + temp.at<Vec3s>(i + 1, j - 1)[c];
                result2[c] /= 4;
                dst.at<Vec3s>(i, j)[c] = result2[c];
            }
        }
    }
    return 0;
}


int sobelY3x3(Mat& src, Mat& dst) {
    Mat temp;
    temp.create(src.size(), CV_16SC3);
    dst.create(src.size(), CV_16SC3);
    Vec3b first_result;
    Vec3i result = { 0,0,0 };
    Vec3i result2 = { 0,0,0 };
    for (int i = 1; i < src.rows - 1; i++) {
        for (int j = 1; j < src.cols - 1; j++) {
            for (int c = 0; c < 3; c++) {
                result[c] = src.at<Vec3b>(i - 1, j - 1)[c] + src.at<Vec3b>(i - 1, j)[c] * 2 + src.at<Vec3b>(i - 1, j + 1)[c];
                result[c] /= 4;
                temp.at<Vec3s>(i, j)[c] = result[c];
            }

        }
    }

    for (int j = 1; j < temp.cols - 1; j++) {
        for (int i = 1; i < temp.rows - 1; i++) {
            for (int c = 0; c < 3; c++) {
                result2[c] = temp.at<Vec3s>(i - 1, j - 1)[c] * -1 + temp.at<Vec3s>(i, j - 1)[c] * 0 + temp.at<Vec3s>(i + 1, j - 1)[c];
                //result2[c] /= 4;
                dst.at<Vec3s>(i, j)[c] = result2[c];

            }
        }
    }
    return 0;
}

int magnitude(Mat& sx, Mat& sy, Mat& dst) {
    dst.create(sx.size(), CV_8UC3);
    int sum;

    for (int i = 0; i < sx.rows; i++) {
        for (int j = 0; j < sx.cols; j++) {
            for (int c = 0; c < 3; c++) {
                sum = sx.at<Vec3s>(i, j)[c] * sx.at<Vec3s>(i, j)[c] + sy.at<Vec3s>(i, j)[c] * sy.at<Vec3s>(i, j)[c];
                dst.at<Vec3b>(i, j)[c] = (unsigned char)sqrt(sum) / sqrt(2);

            }
        }
    }
    return 0;
}



/*for (int o = 0; o < target_features.size(); o++) {
      cout << target_features[o] << endl;
  }*/
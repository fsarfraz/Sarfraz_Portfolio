
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

Mat harris_detector(Mat img) {
	Mat gray, harris_output, norm, harris_out_norm, harris_out_scale;

	// frame to grayscale
	cvtColor(img, gray, COLOR_BGR2GRAY);

	//Using Harris Corners to detect features
	harris_output = Mat::zeros(img.size(), CV_32FC1);
	cornerHarris(gray, harris_output, 3, 3, 0.04);

	//Normalizing image
	normalize(harris_output, harris_out_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	convertScaleAbs(harris_out_norm, harris_out_scale);

	for (int j = 0; j < harris_out_norm.rows; j++) {
		for (int i = 0; i < harris_out_norm.cols; i++) {
			if ((int)harris_out_norm.at<float>(j, i) > 150) {
				circle(img, Point(i, j), 4, Scalar(0, 0, 255), 2, 8, 0);
			}
		}
	}
	return(img);
}

int main()
{
	VideoCapture cap(0);
	Mat frame, gray, harris_output, norm, harris_out_norm, harris_out_scale;

	int k = 0;
	for (;;) {
		cap >> frame;

		harris_detector(frame);

		//Showing robust features in realtime
		imshow("Features", frame);


		char key = waitKey(20);


		if (key == 's') {
			ostringstream save;
			string img = "Image_";
			save << img << k << ".jpg";
			imwrite(save.str(), frame);
			cout << save.str() << " saved" << endl;
			k++;
		}

		// if 'q' is pressed program quits
		if (key == 'q') {
			destroyAllWindows;
			cout << "Closing Program " << endl;
			break;
		}

	}
	
	return 0;
}
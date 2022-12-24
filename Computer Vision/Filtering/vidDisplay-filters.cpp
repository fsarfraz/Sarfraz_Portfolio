
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;


int blur5x5(Mat& src, Mat& dst) {
	Mat temp;
	src.copyTo(temp);
	Vec3b first_result;
	Vec3i result = { 0,0,0 };
	Vec3i result2 = { 0,0,0 };
	for (int i = 2; i < src.rows - 2; i++) {
		for (int j = 2; j < src.cols - 2; j++) {
			for (int c = 0; c < 3; c++) {
				result[c] = src.at<Vec3b>(i - 2, j - 2)[c] + src.at<Vec3b>(i - 2, j - 1)[c] * 2 +
					src.at<Vec3b>(i - 2, j)[c] * 4 + src.at<Vec3b>(i - 2, j + 1)[c] * 2 + src.at<Vec3b>(i - 2, j + 2)[c];
				result[c] /= 10;
				first_result[c] = (unsigned char)result[c];
				temp.at<Vec3b>(i, j)[c] = first_result[c];
			}
			
		}
	}
	temp.copyTo(dst);

	for (int j = 2; j < temp.cols - 2; j++) {
		for (int i = 2; i < temp.rows - 2; i++) {
			for (int c = 0; c < 3; c++) {
				result2[c] = temp.at<Vec3b>(i - 2, j - 2)[c] + temp.at<Vec3b>(i - 1, j - 2)[c] * 2 + temp.at<Vec3b>(i, j - 2)[c] * 4 +
					temp.at<Vec3b>(i + 1, j - 2)[c] * 2 + temp.at<Vec3b>(i + 2, j - 2)[c];
				result2[c] /= 10;
				dst.at<Vec3b>(i, j)[c] = (unsigned char)result2[c];
			}
		}
	}
	return 0;
}

int sobelX3x3(Mat& src, Mat& dst) {
	Mat temp;
	temp.create(src.size(), CV_16SC3);
	dst.create(src.size(), CV_16SC3);
	
	//Vec3b first_result;
	Vec3i result = { 0,0,0 };
	Vec3i result2 = { 0,0,0 };
	for (int i = 1; i < src.rows-1; i++) {
		for (int j = 1; j < src.cols-1; j++) {
			for (int c = 0; c < 3; c++) {
				result[c] = src.at<Vec3b>(i-1, j-1)[c] * -1 + src.at<Vec3b>(i-1, j)[c] * 0 + src.at<Vec3b>(i-1, j + 1)[c];
				//result[c] /= 10;
				temp.at<Vec3s>(i, j)[c] = result[c];
			}

		}
	}

	for (int j = 1; j < temp.cols-1; j++) {
		for (int i = 1; i < temp.rows-1; i++) {
			for (int c = 0; c < 3; c++) {
				result2[c] = temp.at<Vec3s>(i-1, j-1)[c] + temp.at<Vec3s>(i, j-1)[c] * 2 + temp.at<Vec3s>(i + 1, j-1)[c];
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
				result2[c] = temp.at<Vec3s>(i - 1, j - 1)[c]*-1 + temp.at<Vec3s>(i, j - 1)[c]*0 + temp.at<Vec3s>(i + 1, j - 1)[c];
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
			for (int j = 0; j < sx.cols ; j++) {
				for( int c = 0; c<3;c++){
					sum = sx.at<Vec3s>(i, j)[c] * sx.at<Vec3s>(i, j)[c] + sy.at<Vec3s>(i, j)[c] * sy.at<Vec3s>(i, j)[c];
					dst.at<Vec3b>(i, j)[c] = (unsigned char)sqrt(sum) / sqrt(2);
				
				}
			}
		}
		return 0;
}
int blurQ(Mat& src, Mat& dst, int levels) {
	int b = 255 / levels;
	Vec3i result = {0,0,0};
	dst.create(src.size(), CV_8UC3);
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			Vec3b intensity = src.at<Vec3b>(i, j);
			for (int c = 0; c < 3; c++) {
				result[c] = intensity.val[c]/b;
				result[c] *= b;
				dst.at<Vec3b>(i, j)[c] = (unsigned char) result[c];
			}
		}
	}
	return 0;
}

int cartoon(Mat& src, Mat& dst, int levels, int magThreshold) {
	Mat sx, sy, mag, blur, out, out2;
	Vec3i result = { 0,0,0 };
	sobelX3x3(src, sx);
	sobelY3x3(src, sy);
	magnitude(sx, sy, mag);
	
	blur5x5(src, blur);
	blurQ(blur, dst, levels);

	for (int i = 0; i < dst.rows; i++) {
		for (int j = 0; j < dst.cols; j++) {
			for (int c = 0; c < 3; c++) {
				if (mag.at<Vec3b>(i, j)[c] >= magThreshold) {
					dst.at<Vec3b>(i, j)[c] = 0;
				}
			}
		}
	}
	return 0;
}
int mirrorGhost(Mat& src, Mat& dst) {
	Mat img = imread("mirror.png");
	resize(img,img,src.size());
	addWeighted(img, 0.65, src, 0.35, 0,dst);
	return 0;
}

int saveImg(int i) {
	Mat img;
	ostringstream name;
	name << "frame " << i << ".png";
	imwrite(name.str(), img);
	cout << name.str() << " saved " << endl;
	i++;
	return 0;
}


int customgray(Mat& src, Mat& dst) {
	dst = Mat::zeros(src.rows, src.cols, CV_8UC1);
	for (int i = 0; i < src.cols; i++) {
		for (int j = 0; j < src.rows; j++) {
			Vec3b image = src.at<Vec3b>(Point(i, j));
			Scalar gray = dst.at <uchar> (Point(i, j));
			gray = (0.1*image.val[0] + 0.2*image.val[1] + 0.7*image.val[2]);
				dst.at<uchar>(Point(i, j)) = gray.val[0];

		}
	}
	return 0;
}

enum mode{saveImage,regular, grayscale , altgray, gaussBlur, sobelX, sobelY, Magnitude,blurQuantize, cartoonize, effect};

int main() {
	VideoCapture cap(0);
	Mat dst, dst1, dst2, temp1, src;
	Mat frame;
	int task = 0;

	for (;;) {
		cap >> frame;
		char key = waitKey(10);

		if (frame.empty()) {
			printf("Frame is empty\n");
			break;
		}
		if (key == 's') {
			task = saveImage;
			printf("saved image");
		}
		if (key == 'g') {
			task = grayscale;
			printf("gray scale");
		}
		if (key == 'r') {
			task = regular;
			printf("regular");
		}
		if (key == 'h') {
			task = altgray;
			printf("alt gray");
		}
		if (key == 'b') {
			task = gaussBlur;
			printf("gaussblur");
		}
		if (key == 'x') {
			task = sobelX;
			printf("SobelX");
		}
		if (key == 'y') {
			task = sobelY;
			printf("SobelY");
		}
		if (key == 'm') {
			task = Magnitude;
			printf("Magnitude");
		}
		if (key == 'l') {
			task = blurQuantize;
			printf("Blur Quantize");
		}
		if (key == 'c') {
			task = cartoonize;
			printf("cartoon");
		}
		if (key == 'p') {
			task = effect;
			printf("effect");
		}
		if (task == regular) {
			imshow("Video", frame);
		}

		else if (task == grayscale) {
			cvtColor(frame, dst, COLOR_RGBA2GRAY, 0);
			imshow("Video", dst);
		}
		else  if (task == altgray) {
			customgray(frame, dst);
			imshow("Video", dst);
		}
			else if (task == gaussBlur) {
				blur5x5(frame, dst);
				imshow("Video", dst);
			}
			else if (task == sobelX) {
				sobelX3x3(frame, temp1);
				convertScaleAbs(temp1, dst);
				imshow("Video", dst);
			}
			else if (task == sobelY) {
				sobelY3x3(frame, temp1);
				convertScaleAbs(temp1, dst);
				imshow("Video", dst);
			}
			else if (task == Magnitude) {
				sobelX3x3(frame, dst1);
				sobelY3x3(frame, temp1);
				magnitude(dst1, temp1, dst);

				imshow("Video", dst);
			}
			else if (task == blurQuantize) {
				blur5x5(frame, src);
				blurQ(src, dst, 15);
				imshow("Video", dst);
			}
			else if (task == cartoonize) {
				cartoon(frame, dst, 15, 15);
				imshow("Video", dst);
			}
			else if (task == effect) {
				mirrorGhost(frame, dst);
				imshow("Video", dst);
			}
			else
				imshow("Video", frame);

			if (key == 'q') {
				destroyAllWindows;
				cout << "Closing Program " << endl;
				break;
			}
	}
}


		/*if (z == 's') {
			int i;
			saveImg(i =0);
			i++;
		}
		*/
		//imshow("Video capture", frame);


	/*else if (z == 'h') {
		altgreyscale(frame, dst);
		imshow("Video", dst);
	}

	else if (z == 'b') {
		blur5x5(frame, dst);
	}*/

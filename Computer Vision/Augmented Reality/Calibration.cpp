
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main()
{
	TermCriteria termcrit(TermCriteria::MAX_ITER | TermCriteria::EPS, 30, 0.1); // TermCriteria for cornersubpix func
	Size patternsize(9, 6); //checkerboard size
	//Vector intializations
	vector<Point2f> corners;//from chessboard func
	vector<Point2f> lastcorners;
	vector<Vec3f> points;//generated 3D points
	vector<vector <Point2f> >  cornerlist;//list of corner points
	vector<vector<Vec3f> > pointlist;//list of 3D points

	
	VideoCapture cap(0);
	Mat frame,gray,frame2;
	vector<Mat> rvecs, tvecs;// rotation and translation empty mats to be filled
	Size picSize(640, 480);//Calibration image size
	int flags = CALIB_FIX_ASPECT_RATIO + CALIB_FIX_K3 + CALIB_ZERO_TANGENT_DIST + CALIB_FIX_PRINCIPAL_POINT; //flags for clibrate cam function
	Vec<double,5> d_coeff = { 0,0,0,0,0 }; // distortion Coeff
	Matx33d cam_mat(1, 0, frame.cols / 2, 0, 1, frame.rows / 2, 0, 0, 1); //camera matrix


	int k = 0;// counter

	for (;;) {
		cap >> frame;
		cvtColor(frame, gray, COLOR_BGR2GRAY); // single channel input for chessboardcorners func
		
		bool patternWasfound = findChessboardCorners(gray, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE
			| CALIB_CB_FAST_CHECK);

		if (patternWasfound) {
			cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), termcrit);//Corner location refinement
		}

		//drawings corners and displaying
		drawChessboardCorners(frame, patternsize, corners, patternWasfound);
		imshow("Corners", frame);
		
		lastcorners.assign(corners.begin(), corners.end());//corner points

		char key = waitKey(20);
		
		//Keypress 's' for saving Calibration images and generating 3D points for each image
		if (key == 's' && patternWasfound) {
			ostringstream save;
			string calib_img = "CalibrationImage_";
			save << calib_img << k << ".jpg";
			imwrite(save.str(), frame);
			cout << save.str() << " saved" << endl;
			k++;

			cornerlist.push_back(vector<Point2f>(lastcorners));

			//cout << "Corner list size " << cornerlist.size() << endl; debug lines
			
			for (int i = 0; i < patternsize.height; i++) {
				for (int j = 0; j < patternsize.width; j++) {
					points.push_back(Point3f(j,-i, 0.0f));
				}
			}
			pointlist.push_back(vector<Vec3f>(points));
			points.clear();

			//cout << "Point list size " << pointlist.size() << endl; debug lines
			
		}
		//Keypress to calibrate camera, if more than 5 images are saved
		if (key == 'c' && k > 5) {
			cout << "Calibrating ..." << endl;
			cout << "Point list size " << pointlist.size() << endl;
			cout << "Corner list size " << cornerlist.size() << endl;

			float error = calibrateCamera(pointlist, cornerlist,picSize, cam_mat, d_coeff, rvecs, tvecs, flags);

			cout << "Reprojection Error = " << error << "\n Camera Matrix = \n" << cam_mat << "\n Disotrtion Coeff = \n" << d_coeff << endl;
			cout << "Press 'w' if you would like to save Intrinsic Parameters to a file. Press 'r' if you would like to calibrate again." << endl;

		}
		else if (key == 'c' && k < 5) {
			cout << "Not Enough Calibration images are saved. Take more." << endl;
		}

		//Keypress to reset points and corner lists to calibrate camera again
		if (key == 'r') {
			k = 0;
			pointlist.clear();
			cornerlist.clear();
			cout << "Point list and corner lists are clear. Calibrate Again" << endl;
		}

		//Keypress to save intrinsic parameters to a file
		if (key == 'w') {
			FileStorage file("intrinsic_parameters.xml", cv::FileStorage::WRITE);
			// Write to file!
			file << "Camera_Matrix" << cam_mat;
			file << "Distortion_Coefficient_Matrix" << d_coeff;
			cout << "File was saved!" << endl;
		}
		//Keypress to close program
		if (key == 'q') {
			destroyAllWindows;
			cout << "Closing Program " << endl;
			break;
		}

	}

	return 0;
}
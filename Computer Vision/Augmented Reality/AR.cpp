
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

Mat draw_axis(Mat img, Mat rot, Mat trans, Matx33d camera_mat, Vec<double, 5> dist_coeff) {
	vector<Vec3f> axis;
	vector<Point2f> projectedPoints;

	//Points for the Coordinate axes
	axis.push_back(Point3f(3, 0, 0));
	axis.push_back(Point3f(0, -3, 0));
	axis.push_back(Point3f(0, 0, 3));
	axis.push_back(Point3f(0, 0, 0));

	//Projecting ponints to 2D image plane
	projectPoints(axis, rot, trans, camera_mat, dist_coeff, projectedPoints);

	//Drawing the coordinate axis
	line(img, projectedPoints[3], projectedPoints[0], Scalar(255, 0, 0), 3);// z axis
	line(img, projectedPoints[3], projectedPoints[1], Scalar(0, 255, 0), 3);// y axis
	line(img, projectedPoints[3], projectedPoints[2], Scalar(0, 0, 255), 3);// x axis

	return img;

}

Mat draw_house(Mat img, Mat rot, Mat trans, Matx33d camera_mat, Vec<double, 5> dist_coeff) {
	vector<Vec3f> house;
	vector<Point2f> pP;

	//Points for a House drawing
	house.push_back(Point3f(2, 0, 0)); house.push_back(Point3f(6, 0, 0)); house.push_back(Point3f(2, -5, 0)); house.push_back(Point3f(6, -5, 0)); // floor
	house.push_back(Point3f(2, 0, 3)); house.push_back(Point3f(6, 0, 3)); house.push_back(Point3f(2, -5, 3)); house.push_back(Point3f(6, -5, 3)); // walls
	house.push_back(Point3f(4, 0, 5)); house.push_back(Point3f(4, -5, 5)); // roof

	//Projecting ponints to 2D image plane
	projectPoints(house, rot, trans, camera_mat, dist_coeff, pP);

	//Drawing the coordinate axis
	line(img, pP[0], pP[1], Scalar(255, 0, 0), 3); line(img, pP[0], pP[2], Scalar(255, 0, 0), 3); line(img, pP[3], pP[1], Scalar(255, 0, 0), 3); line(img, pP[3], pP[2], Scalar(255, 0, 0), 3); // floor
	line(img, pP[0], pP[4], Scalar(255, 0, 0), 3); line(img, pP[1], pP[5], Scalar(255, 0, 0), 3); line(img, pP[2], pP[6], Scalar(255, 0, 0), 3); line(img, pP[3], pP[7], Scalar(255, 0, 0), 3); // wall height
	line(img, pP[4], pP[5], Scalar(255, 0, 0), 3); line(img, pP[4], pP[6], Scalar(255, 0, 0), 3); line(img, pP[7], pP[5], Scalar(255, 0, 0), 3); line(img, pP[7], pP[6], Scalar(255, 0, 0), 3); //ceiling
	line(img, pP[4], pP[8], Scalar(255, 0, 0), 3); line(img, pP[5], pP[8], Scalar(255, 0, 0), 3); line(img, pP[6], pP[9], Scalar(255, 0, 0), 3); line(img, pP[7], pP[9], Scalar(255, 0, 0), 3);
	line(img, pP[8], pP[9], Scalar(255, 0, 0), 3);


	return img;

}

Mat draw_piece(Mat img, Mat rot, Mat trans, Matx33d camera_mat, Vec<double, 5> dist_coeff, int x, int y, Scalar color) {
	vector<Vec3f> piece;
	vector<Point2f> pP;

	piece.push_back(Point3f(x + 0.2, y - 0.2, 0)); piece.push_back(Point3f(x + 0.8, y - 0.2, 0)); piece.push_back(Point3f(x + 0.2, y - 0.8, 0)); piece.push_back(Point3f(x + 0.8, y - 0.8, 0));
	piece.push_back(Point3f(x + 0.4, y - 0.4, 2)); piece.push_back(Point3f(x + 0.6, y - 0.4, 2)); piece.push_back(Point3f(x + 0.4, y - 0.6, 2)); piece.push_back(Point3f(x + 0.6, y - 0.6, 2));

	//Projecting ponints to 2D image plane
	projectPoints(piece, rot, trans, camera_mat, dist_coeff, pP);

	line(img, pP[0], pP[1], color, 3); line(img, pP[0], pP[2], color, 3); line(img, pP[3], pP[1], color, 3); line(img, pP[3], pP[2], color, 3); // floor
	line(img, pP[0], pP[4], color, 3); line(img, pP[1], pP[5], color, 3); line(img, pP[2], pP[6], color, 3); line(img, pP[3], pP[7], color, 3);
	line(img, pP[4], pP[5], color, 3); line(img, pP[4], pP[6], color, 3); line(img, pP[6], pP[7], color, 3); line(img, pP[7], pP[5], color, 3); // wall height


	return img;

}


enum mode { axis = 1, house = 2, task4 = 3, piece = 4 };
int main()
{
	int task = 0;

	TermCriteria termcrit(TermCriteria::MAX_ITER | TermCriteria::EPS, 30, 0.1); // Term Criteria used
	Size patternsize(9, 6); // checkerboard pattern size
	vector<Point2f> corners;
	vector<Vec3f> points;
	vector<vector<Vec3f> > pointlist;


	VideoCapture cap(0);
	Mat frame, gray, dst, dst1;
	Size picSize(640, 480);
	Vec<double, 5> d_coeff; // distortion Coeff
	Matx33d cam_mat; // Camera Matric
	Mat rvecs(3, 1, DataType<double>::type); // rotation 
	Mat tvecs(3, 1, DataType<double>::type); // translation

	//Reading Intrinsic Parameters from file
	FileStorage file("intrinsic_parameters.xml", FileStorage::READ);
	file["Camera_Matrix"] >> cam_mat;
	file["Distortion_Coefficient_Matrix"] >> d_coeff;
	cout << "file was r4ead" << endl;

	// Generating 3d Points for checkerboard
	for (int i = 0; i < patternsize.height; i++) {
		for (int j = 0; j < patternsize.width; j++) {
			points.push_back(Point3f(j, -i, 0.0f));
		}
	}

	//Looping
	cout << "Press '4' key to show Task 4" << endl;
	cout << "Press 'a' key to show Task 5" << endl;
	cout << "Press 'h' to show Task 6" << endl;
	cout << "Press 'p' to show movable pieces on board" << endl;
	cout << "Control pieces with 'w a s d' and 'i j k l'" << endl;

	//Coordinates for piece
	int x;
	int y;
	x = -1;
	y = -2;
	int x2;
	int y2;
	x2 = 8;
	y2 = -2;


	for (;;) {
		cap >> frame;
		char key = waitKey(20);
		cvtColor(frame, gray, COLOR_BGR2GRAY); //One color channel arg for findchessboard func

		bool patternWasfound = findChessboardCorners(gray, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE
			| CALIB_CB_FAST_CHECK); // Finding corners

		if (patternWasfound) {
			cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), termcrit); //refining corner locations
		}

		//Keypress for task 4, task 5, and task 6
		if (patternWasfound && key == '4') {
			task = task4;
			cout << "Showing Task 4" << endl;
		}
		if (patternWasfound && key == 'x') {
			task = axis;
			cout << "Showing axis" << endl;
		}
		if (patternWasfound && key == 'h') {
			task = house;
			cout << "Showing house" << endl;
		}
		if (patternWasfound && key == 'p') {
			task = piece;
			cout << "Showing piece" << endl;
		}

		if (task == task4) {
			solvePnP(points, corners, cam_mat, d_coeff, rvecs, tvecs);//using solvePnp(task4)
			cout << "rvec: " << rvecs << endl;
			cout << "tvec: " << tvecs << endl;// Printing out rotation and translation matrcies in realtime(task 4)
		}
		if (task == axis) {
			solvePnP(points, corners, cam_mat, d_coeff, rvecs, tvecs);//using solvePnp(task4) 
			draw_axis(frame, rvecs, tvecs, cam_mat, d_coeff);
		}
		if (task == house) {
			solvePnP(points, corners, cam_mat, d_coeff, rvecs, tvecs);//using solvePnp(task4)
			draw_house(frame, rvecs, tvecs, cam_mat, d_coeff);
		}
		if (task == piece) {
			solvePnP(points, corners, cam_mat, d_coeff, rvecs, tvecs);//using solvePnp(task4)

			if (key == 'w') {
				if (y < 1) {
					y += 1;
				}
			}
			if (key == 's') {
				if (y > -5) {
					y += -1;
				}
			}
			if (key == 'd') {
				if (x < 8) {
					x += 1;
				}
			}
			if (key == 'a') {
				if (x > -1) {
					x += -1;
				}
			}
			if (key == 'i') {
				if (y2 < 1) {
					y2 += 1;
				}
			}
			if (key == 'k') {
				if (y2 > -5) {
					y2 += -1;
				}
			}
			if (key == 'l') {
				if (x2 < 8) {
					x2 += 1;
				}
			}
			if (key == 'j') {
				if (x2 > -1) {
					x2 += -1;
				}
			}
			draw_piece(frame, rvecs, tvecs, cam_mat, d_coeff, x, y, Scalar(56, 109, 207));
			draw_piece(frame, rvecs, tvecs, cam_mat, d_coeff, x2, y2, Scalar(207, 56, 64));

		}

		//Showing axes or house drawing in augmented reality
		drawChessboardCorners(frame, patternsize, corners, patternWasfound);
		imshow("Video", frame);

		//Quits program if q key is pressed
		if (key == 'q') {
			destroyAllWindows;
			cout << "Closing Program " << endl;
			break;
		}

	}

	return 0;
}
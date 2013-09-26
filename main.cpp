#include "dataViewer/ReadActivity.h"
#include <opencv2/imgproc/imgproc.hpp>

//
#define JOINT_NUM_2D 15

//////////////////////////////////////////////////////////////////////////

// images and skeleton
double **data;        // [JOINT_NUM][JOINT_DATA_NUM]
int **dataCONF;       // [JOINT_NUM][JOINT_DATA_TYPE_NUM]
double **posData;     // [POS_JOINT_NUM][POS_JOINT_DATA_NUM_TYPE]
int *posDataCONF;     // [POS_JOINT_NUM]

cv::Mat imgRGB, imgDepth;
cv::Mat imgGray;

// acitivity reader
ReadActivity activity;

//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////

// skeleton joints in 2D image

int joints2D[JOINT_NUM_2D][3];
bool joints2DCONF[JOINT_NUM_2D];

//////////////////////////////////////////////////////////////////////////


// allocate resources for data
void initDatas();

void calculateJoints2D();

void getBodyBoundingBox(int& minX, int& maxX, int& minY, int& maxY, int& minZ, int& maxZ);

void showActivity(const string& activityId, const string& dataPathRGBD, const string& dataPathSkeleton);

void showSkeleton(cv::Mat& img);

int main()
{
	string activityId = "0510175411";
//	string activityId = "0510181236";
	string dataPathRGBD =
		"D:\\Liyalong\\cad120\\CAD-120\\CAD-120\\Subject1_rgbd_images\\Subject1_rgbd_images\\arranging_objects";
	string dataPathSkeleton =
		"D:\\Liyalong\\cad120\\CAD-120\\CAD-120\\Subject1_annotations\\Subject1_annotations\\arranging_objects";

	/*
	string dataPathRGBD =
		"D:\\Liyalong\\cad120\\CAD-120\\CAD-120\\Subject1_rgbd_images\\Subject1_rgbd_images\\cleaning_objects";
	string dataPathSkeleton =
		"D:\\Liyalong\\cad120\\CAD-120\\CAD-120\\Subject1_annotations\\Subject1_annotations\\cleaning_objects";
		*/

	showActivity(activityId, dataPathRGBD, dataPathSkeleton);

	return 0;
}

void showActivity(const string& activityId, const string& dataPathRGBD, const string& dataPathSkeleton)
{
	activity.setActivityId(activityId);
	activity.setPath(dataPathRGBD, dataPathSkeleton);

	initDatas();

//	cv::namedWindow("RGB_Window", CV_WINDOW_AUTOSIZE);
//	cv::namedWindow("Depth_Window", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Gray_Window", CV_WINDOW_AUTOSIZE);

	cv::Mat mask;
	cv::namedWindow("Mask_Window", CV_WINDOW_AUTOSIZE);


	while (activity.readNextFrame(data, posData, dataCONF, posDataCONF, imgRGB, imgDepth)) {
		calculateJoints2D();

		// Get mask from depth image and skeleton
		int minX, maxX, minY, maxY, minZ, maxZ;

		getBodyBoundingBox(minX, maxX, minY, maxY, minZ, maxZ);

//		printf("%d, %d\n", minZ, maxZ);

		mask = imgDepth.clone();

		// set pixel value to zero if it is greater than max or lower than min
		for (int i = 0; i < mask.rows; i++) {
			uchar* data = mask.ptr<uchar>(i);

			for (int j = 0; j < mask.cols; j++) { 
				if (data[j] < minZ || data[j] > maxZ) {
					data[j] = 0;
				}
				else {
					data[j] = 255;
				}

				if (i >= minY && i <= maxY && j >= minX && j <= maxX) {
					//
				}
				else {
//					data[j] = 0;
				}
			}
		}

//		showSkeleton(imgRGB);

//		cv::imshow("RGB_Window", imgRGB);
//		cv::imshow("Depth_Window", imgDepth);

		// RGB to gray
		cvtColor(imgRGB, imgGray, CV_RGB2GRAY, 1);

		cv::imshow("Gray_Window", imgGray);
		cv::imshow("Mask_Window", mask);

		cv::waitKey(10);
	}

}

void calculateJoints2D()
{
	for (int i = 0; i < JOINT_NUM; i++) {
		joints2D[i][0] = 2 * (int)(156.8584456124928 + 0.0976862095248 * data[i][9] - 0.0006444357104 * data[i][10] + 0.0015715946682 * data[i][11]);
		joints2D[i][1] = 2 * (int)(125.5357201011431 + 0.0002153447766 * data[i][9] - 0.1184874093530 * data[i][10] - 0.0022134485957 * data[i][11]);
		joints2DCONF[i] = dataCONF[i][1] == 1 ? true : false;

//		printf("%d, %d, %d\n", joints2D[i][0], joints2D[i][1], dataCONF[i][1]);
		joints2D[i][0] = joints2D[i][0] < 0 ? 0 : joints2D[i][0];
		joints2D[i][0] = joints2D[i][0] >= 640 ? 639 : joints2D[i][0];

		joints2D[i][1] = joints2D[i][1] < 0 ? 0 : joints2D[i][1];
		joints2D[i][1] = joints2D[i][1] >= 480 ? 479 : joints2D[i][1];

		joints2D[i][2] = (imgDepth.ptr<uchar>(joints2D[i][1]))[joints2D[i][0]];
	}

	for (int i = JOINT_NUM; i < JOINT_NUM + POS_JOINT_NUM; i++) {
		joints2D[i][0] = 2 * (int)(156.8584456124928 + 0.0976862095248 * posData[i - JOINT_NUM][0] - 0.0006444357104 * posData[i - JOINT_NUM][1] + 0.0015715946682 * posData[i - JOINT_NUM][2]);
		joints2D[i][1] = 2 * (int)(125.5357201011431 + 0.0002153447766 * posData[i - JOINT_NUM][0] - 0.1184874093530 * posData[i - JOINT_NUM][1] - 0.0022134485957 * posData[i - JOINT_NUM][2]);
		joints2DCONF[i] = posDataCONF[i - JOINT_NUM] == 1 ? true : false;

//		printf("%d, %d, %d\n", joints2D[i][0], joints2D[i][1], posDataCONF[i - JOINT_NUM]);
		joints2D[i][0] = joints2D[i][0] < 0 ? 0 : joints2D[i][0];
		joints2D[i][0] = joints2D[i][0] >= 640 ? 639 : joints2D[i][0];

		joints2D[i][1] = joints2D[i][1] < 0 ? 0 : joints2D[i][1];
		joints2D[i][1] = joints2D[i][1] >= 480 ? 479 : joints2D[i][1];

		joints2D[i][2] = (imgDepth.ptr<uchar>(joints2D[i][1]))[joints2D[i][0]];

	}
}

void getBodyBoundingBox(int& minX, int& maxX, int& minY, int& maxY, int& minZ, int& maxZ)
{
	minX = minY = minZ = 1000;
	maxX = maxY = maxZ = 0;

	for (int i = 0; i < JOINT_NUM_2D && joints2DCONF[i]; i++) {
		if (joints2D[i][0] < minX) minX = joints2D[i][0];
		if (joints2D[i][0] > maxX) maxX = joints2D[i][0];

		if (joints2D[i][1] < minY) minY = joints2D[i][1];
		if (joints2D[i][1] > maxY) maxY = joints2D[i][1];

		if (joints2D[i][2] < minZ) minZ = joints2D[i][2];
		if (joints2D[i][2] > maxZ) maxZ = joints2D[i][2];
	}
}

void showSkeleton(cv::Mat& img) {
	for (int i = 0; i < JOINT_NUM; i++) {
		cv::Point jointPos;

		// method to calculate 2D joint's position
		// x = 156.8584456124928 + 0.0976862095248 * x - 0.0006444357104 * y + 0.0015715946682 * z
		// y = 125.5357201011431 + 0.0002153447766 * x - 0.1184874093530 * y - 0.0022134485957 * z

		jointPos.x = 156.8584456124928 + 0.0976862095248 * data[i][9] - 0.0006444357104 * data[i][10] + 0.0015715946682 * data[i][11];
		jointPos.y = 125.5357201011431 + 0.0002153447766 * data[i][9] - 0.1184874093530 * data[i][10] - 0.0022134485957 * data[i][11];

		jointPos.x *= 2;
		jointPos.y *= 2;

		cv::circle(img, jointPos, 3, CV_RGB(0, 0, 255), 3);
	}

	for (int i = 0; i < POS_JOINT_NUM; i++) {
		cv::Point jointPos;

		jointPos.x = 156.8584456124928 + 0.0976862095248 * posData[i][0] - 0.0006444357104 * posData[i][1] + 0.0015715946682 * posData[i][2];
		jointPos.y = 125.5357201011431 + 0.0002153447766 * posData[i][0] - 0.1184874093530 * posData[i][1] - 0.0022134485957 * posData[i][2];

		jointPos.x *= 2;
		jointPos.y *= 2;

		cv::circle(img, jointPos, 3, CV_RGB(255, 0, 0), 3);
	}
}

void initDatas()
{
	data = new double*[JOINT_NUM];
	dataCONF = new int*[JOINT_NUM];

	for (int i = 0; i < JOINT_NUM; i++) {
		data[i] = new double[JOINT_DATA_NUM];
		dataCONF[i] = new int[JOINT_DATA_TYPE_NUM];
	}

	posData = new double*[POS_JOINT_NUM];
	posDataCONF = new int[POS_JOINT_NUM];

	for (int i = 0; i < POS_JOINT_NUM; i++) {
		posData[i] = new double[POS_JOINT_DATA_NUM];
	}

	//
	for (int i = 0; i < JOINT_NUM_2D; i++) {
		joints2DCONF[i] = false;
	}
}
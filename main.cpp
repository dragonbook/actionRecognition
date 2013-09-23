#include "dataViewer/ReadActivity.h"
#include <opencv2/imgproc/imgproc.hpp>

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


// allocate resources for data
void initDatas();

void showActivity(const string& activityId, const string& dataPathRGBD, const string& dataPathSkeleton);

int main()
{
	string activityId = "0510175431";
	string dataPathRGBD =
		"D:\\Liyalong\\cad120\\CAD-120\\CAD-120\\Subject1_rgbd_images\\Subject1_rgbd_images\\arranging_objects";
	string dataPathSkeleton =
		"D:\\Liyalong\\cad120\\CAD-120\\CAD-120\\Subject1_annotations\\Subject1_annotations\\arranging_objects";

	showActivity(activityId, dataPathRGBD, dataPathSkeleton);

	return 0;
}

void showActivity(const string& activityId, const string& dataPathRGBD, const string& dataPathSkeleton)
{
	activity.setActivityId(activityId);
	activity.setPath(dataPathRGBD, dataPathSkeleton);

	initDatas();

	cv::namedWindow("RGB_Window", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Depth_Window", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Gray_Window", CV_WINDOW_AUTOSIZE);


	while (activity.readNextFrame(data, posData, dataCONF, posDataCONF, imgRGB, imgDepth)) {
		cv::imshow("RGB_Window", imgRGB);
		cv::imshow("Depth_Window", imgDepth);

		// RGB to gray
		cvtColor(imgRGB, imgGray, CV_RGB2GRAY, 1);
		cv::imshow("Gray_Window", imgGray);

		cv::waitKey(10);
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
}

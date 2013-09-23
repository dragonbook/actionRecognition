#ifndef READ_ACTIVITY_H
#define READ_ACTIVITY_H

#include "Ulity.h"
#include "SubActivity.h"

struct ObjectAnnotation {
	double x1, y1, x2, y2;			// bounding box
	double t1, t2, t3, t4, t5, t6;	// sift transform matrix
};

class ReadActivity {

public:

	ReadActivity();
	~ReadActivity();

	void setActivityId(const string& id);
	void setPath(const string& dataLocationRGBD, const string& dataLocationSkeleton);		// set all the needed paths

	bool readAnnotation();

	bool readNextFrame(double **data, double **posData, int **dataCONF, int *dataPosCONF,
		cv::Mat& imgRGB, cv::Mat& imgDepth);

	bool readNextFrameRGBD(cv::Mat& imgRGB, cv::Mat& imgDepth);
	bool readNextFrameSkeleton(double **data, double **posData, int **dataCONF, int *dataPosCONF);

	bool readNextFrameObjects(vector<string>& objects);
	bool readNextFrameObjectsAffordances(vector<string>& affordances);
	bool readNextFrameObjectsAnnotations(vector<ObjectAnnotation>& annotations);

	string getActivityType() { return activityType; }
	string getCurrentSubActivityType();

protected:
	bool parseChk(bool chk, bool skeleton);

	void errorMsg(string message, bool exitProgram);
	void errorMsg(string message);

private:
	string activityId;	
	string activityType;

	vector<string> objects;
	vector<SubActivity> subActivities;

	string dataLocationRGBD;
	string dataLocationSkeleton;

	string fileNameRGB;
	string fileNameDepth;

	int currFrameNum;
	int currFrameRGBDNum;

	string fileNameSkeleton;
	ifstream fileSkeleton;

	vector<ifstream *> fileObjectsAnnotations;

	bool isReadAnnotation;

};

#endif

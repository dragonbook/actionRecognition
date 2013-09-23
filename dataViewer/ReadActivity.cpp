#include "ReadActivity.h"

ReadActivity::ReadActivity()
{
	activityId = "";
	activityType = "";
	objects = vector<string>();
	subActivities = vector<SubActivity>();

	currFrameNum = -1;
	currFrameRGBDNum = -1;

	fileObjectsAnnotations = vector<ifstream *>();
}

ReadActivity::~ReadActivity()
{
	if (fileSkeleton) {
		fileSkeleton.close();
	}

	if (! fileObjectsAnnotations.empty()) {
		vector<ifstream *>::iterator iter;

		for (iter = fileObjectsAnnotations.begin(); iter != fileObjectsAnnotations.end(); iter++) {
			(*iter)->close();
		}
	}
}

void ReadActivity::setActivityId(const string& id)
{
	activityId = id;
}

void ReadActivity::setPath(const string& dataLocationRGBD, const string& dataLocationSkeleton)
{
	this->dataLocationRGBD = dataLocationRGBD;
	this->dataLocationSkeleton = dataLocationSkeleton;

	fileNameSkeleton = dataLocationSkeleton + "\\" + activityId + ".txt";
	fileSkeleton.open(fileNameSkeleton);
}

bool ReadActivity::readNextFrameRGBD(cv::Mat& imgRGB, cv::Mat& imgDepth)
{
	if (currFrameRGBDNum <= 0) {
		currFrameRGBDNum = 1;
	}

	stringstream ss;

	ss << currFrameRGBDNum;
	fileNameRGB = dataLocationRGBD + "\\" + activityId + "\\" + "RGB_" + ss.str() + ".png";
	fileNameDepth = dataLocationRGBD + "\\" + activityId + "\\" + "Depth_" + ss.str() + ".png";


	/*
	if (currFrameRGBDNum == 1) {
		printf("\tOpening \"%s\" and so forth..\n", (char*)fileNameRGB.c_str());
		printf("\tOpening \"%s\" and so forth..\n", (char*)fileNameDepth.c_str());
	}
	*/

	imgRGB = cv::imread(fileNameRGB, CV_LOAD_IMAGE_COLOR);
	imgDepth = cv::imread(fileNameDepth, CV_LOAD_IMAGE_GRAYSCALE);

	if (imgRGB.data == NULL) {
		printf("ERROR! Unable to open file %s.\n", (char*)fileNameRGB.c_str());

		return false;
	}

	if (imgDepth.data == NULL) {
		printf("ERROR! Unable to open file %s.\n", (char*)fileNameDepth.c_str());

		return false;
	}

	currFrameRGBDNum++;

	return true;
}

bool ReadActivity::readNextFrameSkeleton(double **data, double **posData, int **dataCONF, int *dataPosCONF)
{
	string line;
	bool fileSkeletonEnded = true;

	if (! fileSkeleton) {
		errorMsg("Open skeleton file failed...\n");
	}

	if (getline(fileSkeleton, line)) {
		fileSkeletonEnded = false;

		stringstream lineStream(line);
		string element;

		int jointCount = 0;
		int jointDataCount = 0;

		int posJointCount = 0;
		int posJointDataCount = 0;

		parseChk(getline(lineStream, element, ','), true);
		currFrameNum = atoi((char *)element.c_str());

		if (element.compare("END") == 0) {
			fileSkeletonEnded = true;

			return false;
		}

		while (getline(lineStream, element, ',')) {
			double e = strtod((char *)element.c_str(), NULL);

			if (jointCount < JOINT_NUM) {
				data[jointCount][jointDataCount] = e;
				jointDataCount++;

				if (jointDataCount == JOINT_DATA_ORI_NUM) {
					parseChk(getline(lineStream, element, ','), true);    // parse conf value
					dataCONF[jointCount][0] = atoi((char *)element.c_str());
				}
				else if (jointDataCount >= JOINT_DATA_NUM) {
					parseChk(getline(lineStream, element, ','), true);
					dataCONF[jointCount][1] = atoi((char *)element.c_str());

					jointCount++;
					jointDataCount = 0;
				}
			}
			else {

				// pos only joints
				if (posJointCount >= POS_JOINT_NUM) {
					errorMsg("PARSING ERROR!!!");
				}

				posData[posJointCount][posJointDataCount] = e;
				posJointDataCount++;

				if (posJointDataCount >= POS_JOINT_DATA_NUM) {
					parseChk(getline(lineStream, element, ','), true);    // parse conf value
					dataPosCONF[posJointCount] = atoi((char *)element.c_str());

					posJointCount++;
					posJointDataCount = 0;
				}
			}
		}

		// check if there is more data in current frame...
		if (getline(lineStream, element, ',')) {
			errorMsg("more data exist in skeleton data...\n");
		}
	}

	if (currFrameNum < 0) {
		errorMsg("file does not exist or empty!!!");
	}

	return !fileSkeletonEnded;
}

bool ReadActivity::readNextFrame(double **data, double **posData, int **dataCONF, int *dataPosCONF,
								 cv::Mat& imgRGB, cv::Mat& imgDepth)
{
	// update currFrameNum when read skeleton file
	bool stateSkeleton = readNextFrameSkeleton(data, posData, dataCONF, dataPosCONF);
	bool stateRGBD = false;
	
	if (stateSkeleton) {
		currFrameRGBDNum = currFrameNum;
		stateRGBD = readNextFrameRGBD(imgRGB, imgDepth);	
	}

	return stateSkeleton && stateRGBD;	
}

bool ReadActivity::readAnnotation()
{
	// Read activityLabel.txt
	ifstream activityLabel;
	string line;
	bool readActivityLable = false;

	string activityLabelPath = dataLocationSkeleton + "//activityLabel.txt";
	activityLabel.open(activityLabelPath);

	while (getline(activityLabel, line)) {
		stringstream linestream(line);
		string element;

		// a line in activityLabel is like:
		// id,activity_id,subject_id,object_id:object_type,
		getline(linestream, element, ',');

		if (activityId == element) {
			getline(linestream, element, ',');
			activityType = element;

			// ignore subject_id
			getline(linestream, element, ',');

			while (getline(linestream, element, ',')) {
				stringstream ss(element);

				string object;
				int objectId;
				char colon;

				ss >> objectId >> colon >> object;
				objects.push_back(object);
			}

			readActivityLable = true;
			break;
		}
	}

	activityLabel.close();

	if (! readActivityLable) {
		return false;
	}

	// Read labeling.txt
	ifstream labeling;
	bool readLabeling = false;

	string labelingPath = dataLocationSkeleton + "//labeling.txt";
	labeling.open(labelingPath);

	while (getline(labeling, line)) {
		stringstream linestream(line);
		SubActivity subActivity;
		string element;

		getline(linestream, element, ',');

		if (activityId == element) {
			getline(linestream, element, ',');	
			subActivity.startFrameNum = atoi((char *)element.c_str());

			getline(linestream, element, ',');	
			subActivity.endFrameNum = atoi((char *)element.c_str());

			getline(linestream, subActivity.subActivityType, ',');

			while (getline(linestream, element, ',')) {
				subActivity.objectsAffordances.push_back(element);
			}

			subActivities.push_back(subActivity);
			readLabeling = true;
		}
	}

	labeling.close();

	if (! readLabeling) {
		return false;
	}

	// setup the object file path
	if (readActivityLable && readLabeling) {
		int count = objects.size();
		char iStr[5];

		for (int i = 1; i <= count; i++) {
			sprintf(iStr, "%d", i);

			string objectFilePath = dataLocationSkeleton + "//" + activityId + "_obj" + iStr + ".txt";
			ifstream* objectAnnotation = new ifstream(objectFilePath);

			fileObjectsAnnotations.push_back(objectAnnotation);
		}
	}

	isReadAnnotation = readActivityLable && readLabeling;

	return readActivityLable && readLabeling;
}

bool ReadActivity::readNextFrameObjects(vector<string>& objects)
{
	if (! isReadAnnotation) {
		return false;
	}

	objects = this->objects;

	return true;
}

bool ReadActivity::readNextFrameObjectsAffordances(vector<string>& affordances)
{
	if (! isReadAnnotation) {
		return false;
	}

	int objectsCount = objects.size();
	bool foundSubActivity = false;

	for (vector<SubActivity>::iterator iter = subActivities.begin(); iter != subActivities.end(); iter++) {
		if (currFrameNum >= iter->startFrameNum && currFrameNum <= iter->endFrameNum) {
			affordances = iter->objectsAffordances;
			foundSubActivity = true;
		}
	}

	if (! foundSubActivity) {
		for (int i = 0; i < objectsCount; i++) {
			affordances.push_back("null");
		}
	}

	return true;
}

bool ReadActivity::readNextFrameObjectsAnnotations(vector<ObjectAnnotation>& annotations)
{
	if (! isReadAnnotation) {
		return false;
	}

	// read object annotation one by one
	vector<ifstream *>::iterator iter;

	for (iter = fileObjectsAnnotations.begin(); iter != fileObjectsAnnotations.end(); iter++) {
		string line;

		if (getline(*(*iter), line)) {

			// read object bounding box and sift transform matrix
			stringstream linestream(line);
			int frameNum, objectId;
			char colon;
			ObjectAnnotation objectAnnotation;

			linestream >> frameNum >> colon >> objectId >> colon;
			linestream >> objectAnnotation.x1 >> colon >> objectAnnotation.y1 >> colon;
			linestream >> objectAnnotation.x2 >> colon >> objectAnnotation.y2 >> colon;
			linestream >> objectAnnotation.t1 >> colon >> objectAnnotation.t2 >> colon;
			linestream >> objectAnnotation.t3 >> colon >> objectAnnotation.t4 >> colon;
			linestream >> objectAnnotation.t5 >> colon >> objectAnnotation.t6 >> colon;

			annotations.push_back(objectAnnotation);
		}
	}

	return true;
}

string ReadActivity::getCurrentSubActivityType()
{
	for (vector<SubActivity>::iterator iter = subActivities.begin(); iter != subActivities.end(); iter++) {
		if (currFrameNum >= iter->startFrameNum && currFrameNum <= iter->endFrameNum) {
			return iter->subActivityType;
		}
	}

	return "null";
}

bool ReadActivity::parseChk(bool chk, bool skeleton)
{
	if (!chk) {
		if (skeleton){
			errorMsg("parsing error. (skeleton)", true);
		}
		else {
			errorMsg("parsing error. (RGBD) - IGNORE THIS ERROR!! (all random dataset will hit this error)", false);
		}

		return false;
	}

	return true;
}

void ReadActivity::errorMsg(string message, bool exitProgram)
{
	cout << "ERROR! " << message << endl;
	printf("\tcurrentFrameNum = %d\n", currFrameNum);
	printf("\tcurrentFrameNum_RGBD = %d\n", currFrameRGBDNum);

	if (exitProgram) {
		exit(1);
	}
}

void ReadActivity::errorMsg(string message)
{
	errorMsg(message, true);
}

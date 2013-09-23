#ifndef SUB_ACTIVITY_H
#define SUB_ACTIVITY_H

#include "Ulity.h"

class SubActivity {
public:
	SubActivity() {
		subActivityType = "";
		startFrameNum = 0;
		endFrameNum = 0;
		objectsAffordances = vector<string>();
	}

public:
	string subActivityType;
	int startFrameNum;
	int endFrameNum;

	vector<string> objectsAffordances;
};

#endif
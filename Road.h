#include<iostream>
#include<string>
#include<algorithm>
#include<thread>
#include<opencv2/opencv.hpp>

#include"Entity.h"
#include"util.h"
using namespace std;
class Road
{
public:
	static int num;
	friend class Video;
	Road(string sourceFile, string procFile, vector<Point> points,  int trajectoryDis = 5, \
		int trajectoryNum = 5, int edge = 100, bool show = true, bool save = false, float iou_low = 0.5, \
		int minLength = 300, int history = 50, int contourArea = 4000, float ratio_low = 1, \
		float ratio_high = 5, int LaneNumber = 3);
	void proc();
	bool finish = false;
private:
	void getRotmat();
	vector<vector<Point>> getContours();
	void getRoad(Mat m);
	void getBack(Mat& m);
	void getEnList(vector<vector<Point>> contours);
	void detect();
	void draw();
	int visual(Mat &m);
	void release();
	
private:
	int minLength;
	float iou_low;
	int history;
	int contourArea;
	float ratio_low;
	float ratio_high;
	int LaneNumber;
	int trajectoryDis;
	int trajectoryNum;
	int edge;
	//道路参数
	int mNum = 0;
	float scale;
	Mat road, mBack, roiInm;
	string dir;
	vector<Point> points;
	RotatedRect rect;
	Mat rot_mat;
	Mat invert_mat;
	Ptr<BackgroundSubtractor> pMOG2;
	Mat fgMaskMOG2;
	//实体参数
	int enId = 0;
	vector<Entity> enList;
	int pass;

	int roadId;
	bool show;

	string sourceFile;
	string procFile;
	bool save;
	VideoCapture cap;
	VideoWriter writer;
};

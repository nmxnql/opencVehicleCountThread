#pragma once
#include<vector>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
class Entity
{
public:
	Entity(int id, Rect r, bool stat = true);
	float overlap(Rect rt);
	void add(Rect rt);
	int id;
	vector<Rect> sunRect;
	bool stat;
};
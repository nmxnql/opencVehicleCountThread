#include<iostream>
#include<string>
#include<algorithm>
#include<thread>

#include"Entity.h"
#include"Road.h"
#include"util.h"
using namespace std;

int main()
{
	double time_Start = (double)clock();  //程序运行计时开始

	string sourceFile = "..//..//video//test2.avi";
	string procFile = "..//..//video//proc_test2.avi";
	
	int points_t1[][2] = { { 345, 302 }, { 327, 259 }, { 465, 205 }, { 482, 248 } };
	int points_t2[][2] = { { 196, 426 } ,{ 174, 373 }, { 358, 298 },{ 379, 350 } };
	vector<Point> points;
	for (int i = 0; i < sizeof(points_t1) / sizeof(points_t1[0]); i++)
	{
		points.push_back(Point(points_t1[i][0], points_t1[i][1]));
	}
	Road r1 = Road(sourceFile, procFile, points);
	points.clear();
	for (int i = 0; i < sizeof(points_t2) / sizeof(points_t2[0]); i++)
	{
		points.push_back(Point(points_t2[i][0], points_t2[i][1]));
	}
	Road r2 = Road(sourceFile, procFile,points, 3, 3, 100);
	double start_proc;
	start_proc = (double)clock();
	thread t1(&(Road::proc), &r1);
	thread t2(&(Road::proc), &r2);
	
	t1.detach();
	t2.detach();
	while (1)
	{
		if (r1.finish && r2.finish) break;
	};
	cout << getTime(start_proc, "proc", "min");
	return 1;
}

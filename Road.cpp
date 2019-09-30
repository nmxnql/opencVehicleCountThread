#include "Road.h"
int Road::num = 0;
Road::Road(string sourceFile, string procFile, vector<Point> points, int trajectoryDis , int trajectoryNum , int edge , bool show , bool save, float iou_low , \
	int minLength, int history, int contourArea, float ratio_low, \
	float ratio_high, int LaneNumber) :
	sourceFile(sourceFile), procFile(procFile), points(points), trajectoryDis(trajectoryDis), trajectoryNum(trajectoryNum), edge(edge), show(show), \
	minLength(minLength), iou_low(iou_low), history(history), contourArea(contourArea), \
	ratio_low(ratio_low), ratio_high(ratio_high), LaneNumber(LaneNumber)
{
	pMOG2 = createBackgroundSubtractorMOG2(200);
	roadId = ++num;
	cap = VideoCapture(sourceFile);
	if (procFile != "")
	{
		int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		writer = VideoWriter(procFile, CV_FOURCC('M', 'J', 'P', 'G'), cap.get(CAP_PROP_FPS), Size(frame_width, frame_height));
	}
}

void Road::getRotmat()
{
	//获取仿射变换矩阵
	rect = minAreaRect(points);
	double x0, y0, alpha;
	x0 = rect.center.x;
	y0 = rect.center.y;
	alpha = -rect.angle * CV_PI / 180;
	Mat rot_mat_tmp(2, 3, CV_32FC1);
	rot_mat_tmp = getRotationMatrix2D(rect.center, rect.angle, 1.0);
	rot_mat = rot_mat_tmp;
	Mat invert_mat_tmp(2, 3, CV_32FC1);
	invertAffineTransform(rot_mat, invert_mat_tmp);
	invert_mat = invert_mat_tmp;
	//获取缩放尺度
	if (rect.size.width > rect.size.height)
	{
		scale = rect.size.height * 1.0 / minLength;
	}
	else
	{
		scale = rect.size.width * 1.0 / minLength;
	}
}

vector<vector<Point>> Road::getContours()
{
	Mat binary;
	threshold(fgMaskMOG2, binary, 244, 255, THRESH_BINARY);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	Mat erode_t;
	erode(binary, erode_t, element);
	Mat dilate_t;
	element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	dilate(erode_t, dilate_t, element);

	int i = 0;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	for (i = 0; i < LaneNumber; i++)
	{
		Mat subdilate_t = Mat::zeros(dilate_t.rows, dilate_t.cols, dilate_t.depth());
		if (dilate_t.cols > dilate_t.rows)
		{
			int x1 = 0;
			int y1 = i * dilate_t.rows / LaneNumber;
			int w = dilate_t.cols;
			int h = dilate_t.rows / LaneNumber;
			Rect rect = Rect(x1, y1, w, h);
			dilate_t(rect).copyTo(subdilate_t(rect));

			vector<vector<Point>> subcontours;
			vector<Vec4i> hierarchy;
			findContours(subdilate_t, subcontours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			contours.insert(contours.end(), subcontours.begin(), subcontours.end());
		}
	}

	return contours;
}

void Road::getRoad(Mat m)
{
	//获取道路和背景分离的图片
	Mat maskRoad = Mat(m.rows, m.cols, CV_8UC3, Scalar(0, 0, 0));
	vector<vector<Point> > vpts;
	vpts.push_back(points);
	fillPoly(maskRoad, vpts, Scalar(255, 255, 255), 8, 0);
	Mat mRoad;
	bitwise_and(m, maskRoad, mRoad);
	Mat maskBack;
	bitwise_not(maskRoad, maskBack);
	bitwise_and(m, maskBack, mBack);
	//道路校正
	Size dst_sz(m.rows, m.cols);
	warpAffine(mRoad, roiInm, rot_mat, dst_sz);
	//提取道路
	int x1 = int(rect.center.x - rect.size.width / 2);
	int y1 = int(rect.center.y - rect.size.height / 2);
	Mat roadOri = roiInm(Rect(x1, y1, int(rect.size.width), int(rect.size.height)));

	int w = int(roadOri.cols / scale);
	int h = int(roadOri.rows / scale);
	resize(roadOri, road, Size(w, h));

}

void Road::getEnList(vector<vector<Point>> contours)
{
	vector<int> updateId;
	vector<Entity> newenList;
	for (auto c : contours)
	{
		Rect r = boundingRect(c);
		bool flag = false;
		if (r.width * r.height > contourArea)
		{
			if (dir == "left" || dir == "right")
			{
				float ratio = r.width * 1.0 / r.height;
				if (ratio > ratio_low&& ratio < ratio_high)
				{
					flag = true;
				}
			}
			else if (dir == "top" || dir == "down")
			{
				float ratio = r.height * 1.0 / r.width;
				if (ratio > ratio_low&& ratio < ratio_high)
				{
					flag = true;
				}
			}
			else
			{
				flag = true;
			}
		}
		if (flag)
		{
			if (enList.size() == 0)
			{
				newenList.push_back(Entity(enId, r));
				updateId.push_back(enId);
				enId++;

			}
			else
			{
				int max_index = -1;
				float max_iou = -1.0;
				for (int i = 0; i < enList.size(); i++)
				{
					float iou = enList[i].overlap(r);
					if (iou > iou_low&& iou > max_iou)
					{
						max_iou = iou;
						max_index = i;
					}
				}

				if (max_index == -1)
				{
					newenList.push_back(Entity(enId, r));
					updateId.push_back(enId);
					enId++;
				}
				else
				{
					enList[max_index].add(r);
					updateId.push_back(enList[max_index].id);
				}

			}
		}
	}

	enList.insert(enList.end(), newenList.begin(), newenList.end());
	for (vector<Entity>::iterator en = enList.begin(); en != enList.end();)
	{
		if (find(updateId.begin(), updateId.end(), en->id) == updateId.end())
		{
			en = enList.erase(en);
		}
		else
		{
			en++;
		}
	}
}

void Road::detect()
{
	pMOG2->apply(road, fgMaskMOG2);
	vector<vector<Point>> contours = getContours();
	if (mNum < history)
	{
		getEnList(contours);
	}
	else if (mNum == history)
	{
		int left, right, top, down;
		left = right = top = down = 0;
		for (auto en : enList)
		{
			if (en.sunRect.size() > trajectoryNum)
			{
				Rect last = *(en.sunRect.rbegin());
				Rect first = *(en.sunRect.begin());
				int dx = last.x - first.x;
				int dy = last.y - first.y;

				if (road.cols > road.rows)
				{
					if (dx > 0) right++;
					if (dx < 0) left++;
				}
				else
				{
					if (dy > 0) down++;
					if (dy < 0) top++;
				}
			}
		}
		if (road.cols > road.rows)
		{
			if (left > right) dir = "left";
			else dir = "right";
		}
		else
		{
			if (down > top) dir = "down";
			else dir = "top";
		}
	}
	else
	{
		getEnList(contours);

		for (auto en = enList.begin(); en != enList.end();)
		{
			Rect last = *(en->sunRect.rbegin());
			Rect first = *(en->sunRect.begin());
			if (dir == "right")
			{
				if (last.x + edge >= road.cols && first.x + edge < road.cols && en->sunRect.size() > trajectoryNum&& last.x - first.x > trajectoryDis)
				{
					pass++;
					en = enList.erase(en);
					cout << "right passNum :" << pass << endl;
				}
				else
				{

					en++;
				}
			}
			if (dir == "left")
			{
				if (last.x - edge <= 0 && first.x - edge > 0 && en->sunRect.size() > trajectoryNum&& first.x - last.x > trajectoryDis)
				{
					pass++;
					en = enList.erase(en);
					cout << "left passNum :" << pass << endl;
				}
				else
				{
					en++;
				}
			}
			if (dir == "top")
			{
				if (last.y - edge <= 0 && first.y - edge > 0 && en->sunRect.size() > trajectoryNum&& first.y - last.y > trajectoryDis)
				{
					pass++;
					en = enList.erase(en);
					cout << "passNum :" << pass << endl;
				}
				else
				{
					en++;
				}
			}
			if (dir == "down")
			{
				if (last.y + edge >= road.rows && first.y - edge < road.rows && en->sunRect.size() > trajectoryNum&& last.y - first.y > trajectoryDis)
				{
					pass++;
					en = enList.erase(en);
					cout << "passNum :" << pass << endl;
				}
				else
				{
					en++;
				}
			}
		}
	}
}

void Road::getBack(Mat& m)
{
	//road_FIRST = cv2.resize(road_FIRST, ((int)(x2_FIRST - x1_FIRST), (int)(y2_FIRST - y1_FIRST)), interpolation = cv2.INTER_CUBIC)

	//masked1_FIRST[y1_FIRST:y2_FIRST, x1_FIRST : x2_FIRST] = road_FIRST
	//masked1_FIRST = cv2.warpAffine(masked1_FIRST, invert_rot_mat_FIRST, (cols, rows))
	int w = int(road.cols * scale);
	int h = int(road.rows * scale);
	Mat roadOri;
	resize(road, roadOri, Size(w, h));
	Rect rect_t = Rect(rect.center.x - w / 2, rect.center.y - h / 2, w, h);
	roadOri.copyTo(roiInm(rect_t));
	warpAffine(roiInm, roiInm, invert_mat, Size(roiInm.rows, roiInm.cols));
	bitwise_or(roiInm, mBack, m);
}

void Road::draw()
{
	for (auto en : enList)
	{
		rectangle(road, *(en.sunRect.rbegin()), Scalar(0, 255, 0), 2, LINE_8, 0);
		putText(road, to_string(en.id), Point((*(en.sunRect.rbegin())).x, (*(en.sunRect.rbegin())).y), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 1, 8, false);
	}
	Point p1, p2;
	int w, h;
	w = road.cols / 7, h = road.rows / 7;
	if (dir == "right")
	{
		Mat zero = Mat::zeros(h, w, CV_8UC3);
		string text = "passed: " + to_string(pass);
		putText(zero, text, Point(5, 15), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1, 8, false);
		Rect rect = Rect(0, 0, w, h);
		zero.copyTo(road(rect));
		p1 = Point(road.cols - edge, 0), p2 = Point(road.cols - edge, road.rows);
	}
	if (dir == "left")
	{
		Mat zero = Mat::zeros(h, w, CV_8UC3);
		string text = "passed: " + to_string(pass);
		putText(zero, text, Point(5, 15), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1, 8, false);
		Rect rect = Rect(road.cols - w, 0, w, h);
		zero.copyTo(road(rect));
		p1 = Point(edge, 0), p2 = Point(edge, road.rows);
	}
	if (dir == "top")
	{
		p1 = Point(0, edge), p2 = Point(road.cols, edge);
	}
	if (dir == "dwon")
	{
		p1 = Point(0, road.rows - edge), p2 = Point(road.cols, road.rows - edge);
	}
	line(road, p1, p2, cv::Scalar(0, 255, 0), 3, 4);

	
	
	

	
}

int Road::visual(Mat &m)
{
	if (show)
	{
		imshow(to_string(roadId), road);
		if (waitKey(1) > 0) return 0;
	}
	return 1;
}


void Road::release()
{
	cap.release();
	if (procFile != "")
	{
		writer.release();
	}
}

void Road::proc()
{
	if (!cap.isOpened())
	{
		cout << "open video error!";
		return;
	}
	getRotmat();
	Mat m;
	while (cap.read(m))
	{
		++mNum;
		double start_getRoad = (double)clock();
		getRoad(m);
		//cout << getTime(start_getRoad, "getRoad", "ms");
		double start_detect = (double)clock();
		detect();
		//cout << getTime(start_detect, "detect", "ms");
		draw();
		double start_getBack = (double)clock();
		getBack(m);
		//cout << getTime(start_getBack, "getBack", "ms");
		double start_visual = (double)clock();
		int res = visual(m);
		//cout << getTime(start_visual, "visual", "ms");
		if (res == 0) break;
	}
	release();
	finish = true;
}

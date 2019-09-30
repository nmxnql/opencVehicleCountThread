#include<algorithm>
#include "Entity.h"
using namespace std;
using namespace cv;

Entity::Entity(int id, Rect r, bool stat)
{
	this->id = id;
	sunRect.push_back(r);
	this->stat = stat;
}
float Entity::overlap(Rect rt)
{
	Rect r = sunRect[sunRect.size() - 1];
	int endx = max(r.x + r.width, rt.x + rt.width);
	int startx = min(r.x, rt.x);
	int width = r.width + rt.width - (endx - startx);

	int endy = max(r.y + r.height, rt.y + rt.height);
	int starty = min(r.y, rt.y);
	int height = r.height + rt.height - (endy - starty);

	if (width <= 0 || height <= 0)
	{
		return 0;
	}
	else
	{
		float Area = width * height;
		float Area1 = r.width * rt.height;
		float Area2 = r.width * rt.height;
		float ratio = Area / (Area1 + Area2 - Area);
		return ratio;
	}
}

void Entity::add(Rect rt)
{
	sunRect.push_back(rt);
	stat = true;
}

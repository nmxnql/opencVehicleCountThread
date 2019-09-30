#include"util.h"
ostream& operator<<(ostream& out, Entity& en)    

{
	cout << "id :" << en.id << endl;
	cout << "stat :" << en.stat << endl;
	cout << "sunRect :";
	for (auto rect : en.sunRect)
	{
		cout << rect << endl;
	}
	return out;
}

string getTime(double start, const string name, const string ext = "ms")
{
	double end = (double)clock();
	int use = (end - start);
	string s;
	if (ext == "ms") s = name + " time is : " + to_string(use) + "ms\n";
	if (ext == "s") s = name + " time is : " + to_string(use / 1000) + "s\n";
	if (ext == "min")
	{
		use = (end - start) / 1000.0;
		int minute = use / 60;
		int second = use % 60;
		s = name + " time is :" + to_string(minute) + "min" + to_string(second) + "s\n";
	}
	return s;
}
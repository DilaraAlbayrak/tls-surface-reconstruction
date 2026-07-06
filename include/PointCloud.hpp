#pragma once
#include <vector>

struct PointData
{
	float x, y, z;
	int intensity;
	float r, g, b;	
};

class PointCloud
{
public:
	std::vector<PointData> points;
};
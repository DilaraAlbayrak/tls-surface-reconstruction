#pragma once
#include <vector>

struct PointData
{
	float x, y, z;
	int intensity;
	float r, g, b;	
};

struct PointCloud
{
	std::vector<PointData> points;
};
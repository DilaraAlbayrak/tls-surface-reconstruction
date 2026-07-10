#pragma once
#include <vector>

struct PointData
{
	float x, y, z;
	int intensity;
	float r, g, b;	
};

struct NormalData
{
	float nx, ny, nz;
};

struct PointCloud
{
	std::vector<PointData> points;
	std::vector<NormalData> normals;
};
#pragma once
#include <string>
#include "PointCloud.hpp"

class PointCloudIO
{
public:
	static bool loadFromTxt(const std::string& filepath, PointCloud& pointCloud);
	static bool loadFromBin(const std::string& filepath, PointCloud& pointCloud);
	static bool saveToBin(const std::string& filepath, const PointCloud& pointCloud);

	// Orchestrates the check-load-or-convert logic
	static bool loadOrCreateBin(const std::string& txtFilePath, const std::string& binFilePath, PointCloud& pointCloud);
};

#pragma once
#include <string>
#include "PointCloud.hpp"

namespace PointCloudIO
{
	bool loadFromTxt(const std::string& filepath, PointCloud& pointCloud);
	bool loadFromBin(const std::string& filepath, PointCloud& pointCloud);
	bool saveToBin(const std::string& filepath, const PointCloud& pointCloud);

	// Orchestrates the check-load-or-convert logic
	bool loadOrCreateBin(const std::string& txtFilePath, const std::string& binFilePath, PointCloud& pointCloud);
	
	bool saveToPLY(const std::string& filepath, const PointCloud& pointCloud);
	bool loadFromPLY(const std::string& filepath, PointCloud& cloud);
};

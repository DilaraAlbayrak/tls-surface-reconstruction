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
};

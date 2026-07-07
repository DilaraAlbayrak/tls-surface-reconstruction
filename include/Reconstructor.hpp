#pragma once
#include "PointCloud.hpp"

namespace Reconstructor
{
	void process(const PointCloud& cloud);

	PointCloud voxelGridDownsample(const PointCloud& inputCloud, float voxelSize);
};

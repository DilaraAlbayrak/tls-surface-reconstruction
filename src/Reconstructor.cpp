#include "Reconstructor.hpp"
#include <iostream>
#include <unordered_map>
#include <cmath>
#include "nanoflann.hpp"

namespace
{
	struct VoxelIndex
	{
		int x, y, z;

		bool operator==(const VoxelIndex& other) const
		{
			return x == other.x && y == other.y && z == other.z;
		}
	};

	struct VoxelHash
	{
		std::size_t operator()(const VoxelIndex& v) const
		{
			const std::size_t h1 = std::hash<int>()(v.x);
			const std::size_t h2 = std::hash<int>()(v.y);
			const std::size_t h3 = std::hash<int>()(v.z);

			return ((h1 ^ (h2 << 1)) >> 1) ^ (h3 << 1);
		}
	};

	struct VoxelAccumulator
	{
		float x = 0.0f, y = 0.0f, z = 0.0f;
		float r = 0.0f, g = 0.0f, b = 0.0f;
		long long intensity = 0;
		int count = 0;

		void addPoint(const PointData& pt)
		{
			x += pt.x;
			y += pt.y;
			z += pt.z;
			r += pt.r;
			g += pt.g;
			b += pt.b;
			intensity += pt.intensity;
			count++;
		}

		PointData getCentroid() const
		{
			PointData centroid;
			centroid.x = x / count;
			centroid.y = y / count;
			centroid.z = z / count;
			centroid.r = r / count;
			centroid.g = g / count;
			centroid.b = b / count;
			centroid.intensity = static_cast<int>(intensity / count);
			return centroid;
		}
	};

	struct PointCloudAdaptor
	{
		const PointCloud& cloud;
		PointCloudAdaptor(const PointCloud& c) : cloud(c) {}
		inline size_t kdtree_get_point_count() const { return cloud.points.size(); }
		inline float kdtree_get_pt(const size_t idx, const size_t dim) const
		{
			if (dim == 0) return cloud.points[idx].x;
			else if (dim == 1) return cloud.points[idx].y;
			else return cloud.points[idx].z;
		}
		template <class BBOX>
		bool kdtree_get_bbox(BBOX&) const { return false; }
	};
}

namespace Reconstructor
{
	PointCloud voxelGridDownsample(const PointCloud& inputCloud, float voxelSize)
	{
		if (inputCloud.points.empty() || voxelSize <= 0.0f)
		{
			return inputCloud;
		}

		std::unordered_map<VoxelIndex, VoxelAccumulator, VoxelHash> voxelGrid;

		for (const auto& pt : inputCloud.points)
		{
			VoxelIndex idx;
			idx.x = static_cast<int>(std::floor(pt.x / voxelSize));
			idx.y = static_cast<int>(std::floor(pt.y / voxelSize));
			idx.z = static_cast<int>(std::floor(pt.z / voxelSize));

			voxelGrid[idx].addPoint(pt);
		}

		PointCloud downsampledCloud;

		downsampledCloud.points.reserve(voxelGrid.size());

		for (const auto& pair : voxelGrid)
		{
			downsampledCloud.points.push_back(pair.second.getCentroid());
		}

		return downsampledCloud;
	}

	void process(const PointCloud& cloud)
	{
		if (cloud.points.empty())
		{
			std::cerr << "Error: Point cloud is empty. Cannot process.\n";
			return;
		}

		std::cout << "Processing point cloud with " << cloud.points.size() << " points...\n";

		const float VOXEL_SIZE = 0.05f;
		std::cout << "Running Voxel Grid Downsampling with voxel size: " << VOXEL_SIZE << "m...\n";

		PointCloud filteredCloud = voxelGridDownsample(cloud, VOXEL_SIZE);

		std::cout << "Downsampled point cloud size: " << filteredCloud.points.size() << " points.\n";
	}
}
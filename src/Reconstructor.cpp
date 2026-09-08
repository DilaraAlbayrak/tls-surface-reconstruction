#include "Reconstructor.hpp"
#include "PointCloudIO.hpp"
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <vector>
#include <queue>
#include "nanoflann.hpp"
#include <Eigen/Dense>

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

	struct PropagationEdge
	{
		float cost;
		size_t fromIdx; // The index of the point from which the normal is being propagated
		size_t toIdx;   // The index of the point to which the normal is being propagated

		bool operator>(const PropagationEdge& other) const
		{
			return cost > other.cost; // Min-Heap based on cost
		}
	};

	using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
		PointCloudAdaptor, 3>;

	NormalData computeNormalPCA(const PointCloud& cloud, const std::vector<unsigned int>& neighbours)
	{
		if (neighbours.size() < 3) return { 0.0f, 0.0f, 1.0f };

		Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
		for (size_t idx : neighbours)
		{
			centroid.x() += cloud.points[idx].x;
			centroid.y() += cloud.points[idx].y;
			centroid.z() += cloud.points[idx].z;
		}
		centroid /= static_cast<float>(neighbours.size());

		Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
		for (size_t idx : neighbours)
		{
			Eigen::Vector3f pt(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);
			Eigen::Vector3f centered = pt - centroid;
			covariance += centered * centered.transpose();
		}
		covariance /= static_cast<float>(neighbours.size());

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);

		if (solver.info() != Eigen::Success) return { 0.0f, 0.0f, 1.0f };

		Eigen::Vector3f normal = solver.eigenvectors().col(0); // Smallest eigenvalue corresponds to the normal, i.e. the direction of least variance
		
		// Return unoriented PCA normal, MST will orient it later
		return { normal.x(), normal.y(), normal.z() };
	}

	void propagateNormalMST(PointCloud& cloud, const KDTree& kdTree, size_t k_neighbours)
	{
		size_t numPoints = cloud.points.size();
		if (numPoints == 0) return;

		std::vector<bool> visited(numPoints, false);
		std::priority_queue<PropagationEdge, std::vector<PropagationEdge>, std::greater<PropagationEdge>> minHeap;

		// Calculate global centroid to determine a reliable outward direction for the first normal
		Eigen::Vector3f globalCentroid(0.0f, 0.0f, 0.0f);

		for (const auto& pt : cloud.points)
		{
			globalCentroid.x() += pt.x;
			globalCentroid.y() += pt.y;
			globalCentroid.z() += pt.z;
		}
		globalCentroid /= static_cast<float>(numPoints);

		std::vector<unsigned int> neighbourIndices(k_neighbours);
		std::vector<float> neighbourDistances(k_neighbours);

		// Loop through points to ensure disconnected components are also processed
		for (size_t i = 0; i < numPoints; ++i)
		{
			if (visited[i]) continue;
			
			size_t seedIdx = i;

			// if this is the first point, find the point farthest from the global centroid to use as the initial seed for MST
			if (i == 0)
			{
				float maxDistSq = -1.0f;

				for (size_t j = 0; j < numPoints; ++j)
				{
					Eigen::Vector3f pt(cloud.points[j].x, cloud.points[j].y, cloud.points[j].z);
					float distSq = (pt - globalCentroid).squaredNorm();
					if (distSq > maxDistSq)
					{
						maxDistSq = distSq;
						seedIdx = j;
					}
				}
			}

			visited[seedIdx] = true;

			// Orient the seed normal to point outward from the global centroid
			Eigen::Vector3f seedPt(cloud.points[seedIdx].x, cloud.points[seedIdx].y, cloud.points[seedIdx].z);
			Eigen::Vector3f outwardDir = seedPt - globalCentroid;
			Eigen::Vector3f seedNormal(cloud.normals[seedIdx].nx, cloud.normals[seedIdx].ny, cloud.normals[seedIdx].nz);

			if (seedNormal.dot(outwardDir) < 0.0f)
			{
				// Flip the normal
				cloud.normals[seedIdx].nx *= -1.0f;
				cloud.normals[seedIdx].ny *= -1.0f;
				cloud.normals[seedIdx].nz *= -1.0f;
				seedNormal = -seedNormal;
			}

			// Add the seed point's neighbors to the priority queue
			float queryPoint[3] = { cloud.points[seedIdx].x, cloud.points[seedIdx].y, cloud.points[seedIdx].z };
			size_t numFound = kdTree.knnSearch(queryPoint, k_neighbours, neighbourIndices.data(), neighbourDistances.data());

			for (size_t j = 0; j < numFound; ++j)
			{
				size_t neighborIdx = neighbourIndices[j];
				if (!visited[neighborIdx])
				{
					Eigen::Vector3f neighborNormal(cloud.normals[neighborIdx].nx, cloud.normals[neighborIdx].ny, cloud.normals[neighborIdx].nz);
					// Calculate cost as the angle between normals (1 - dot product), parallel normals have cost 0, opposite normals have cost 2
					float cost = 1.0f - std::abs(seedNormal.dot(neighborNormal));
					minHeap.push({ cost, seedIdx, neighborIdx });
				}
			}

			// Process MST propagation
			while (!minHeap.empty())
			{
				PropagationEdge edge = minHeap.top();
				minHeap.pop();
				
				size_t currentIdx = edge.toIdx;
				if (visited[currentIdx]) continue;

				visited[currentIdx] = true;

				Eigen::Vector3f fromNormal(cloud.normals[edge.fromIdx].nx, cloud.normals[edge.fromIdx].ny, cloud.normals[edge.fromIdx].nz);
				Eigen::Vector3f currentNormal(cloud.normals[currentIdx].nx, cloud.normals[currentIdx].ny, cloud.normals[currentIdx].nz);

				// If normals are pointing in opposite directions, flip the current normal
				if (fromNormal.dot(currentNormal) < 0.0f)
				{
					cloud.normals[currentIdx].nx *= -1.0f;
					cloud.normals[currentIdx].ny *= -1.0f;
					cloud.normals[currentIdx].nz *= -1.0f;
					currentNormal = -currentNormal;
				}

				// Add unvisited neighbors of the current point to the priority queue
				float queryPoint[3] = { cloud.points[currentIdx].x, cloud.points[currentIdx].y, cloud.points[currentIdx].z };
				numFound = kdTree.knnSearch(queryPoint, k_neighbours, neighbourIndices.data(), neighbourDistances.data());

				for (size_t j = 0; j < numFound; ++j)
				{
					size_t neighborIdx = neighbourIndices[j];
					if (!visited[neighborIdx])
					{
						Eigen::Vector3f neighborNormal(cloud.normals[neighborIdx].nx, cloud.normals[neighborIdx].ny, cloud.normals[neighborIdx].nz);
						float cost = 1.0f - std::abs(currentNormal.dot(neighborNormal));
						minHeap.push({ cost, currentIdx, neighborIdx });
					}
				}
			}
		}
	}
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

		std::cout << "Building KD-Tree for nearest neighbor search...\n";

		PointCloudAdaptor adaptor(filteredCloud);

		KDTree kdTree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(15)); // 3 is for dimension, 15 is the maximum number of points in a leaf node
		kdTree.buildIndex();

		std::cout << "KD-Tree built successfully.\n";

		const size_t K_NEIGHBOURS = 15;
		filteredCloud.normals.resize(filteredCloud.points.size());

		std::vector<unsigned int> neighbourIndices(K_NEIGHBOURS);
		std::vector<float> neighbourDistances(K_NEIGHBOURS);
		size_t failedPCA = 0;

		std::cout << "Calculating normals using PCA with K=" << K_NEIGHBOURS << " nearest neighbors...\n";

		for (size_t i = 0; i < filteredCloud.points.size(); ++i)
		{
			const auto& pt = filteredCloud.points[i];
			float queryPoint[3] = { pt.x, pt.y, pt.z };

			size_t numFound = kdTree.knnSearch(
				queryPoint,
				K_NEIGHBOURS,
				neighbourIndices.data(),
				neighbourDistances.data()
			);

			if (numFound > 0)
			{
				neighbourIndices.resize(numFound);
				filteredCloud.normals[i] = computeNormalPCA(filteredCloud, neighbourIndices);
				neighbourIndices.resize(K_NEIGHBOURS);

				// Check if PCA failed to compute a valid normal (i.e., returned the default normal)
				if (filteredCloud.normals[i].nx == 0.0f &&
					filteredCloud.normals[i].ny == 0.0f &&
					filteredCloud.normals[i].nz == 1.0f)
				{
					failedPCA++;
				}
			}
			else
			{
				filteredCloud.normals[i] = { 0.0f, 0.0f, 1.0f };
				failedPCA++;
			}
		}

		// Print warning for degenerate normals
		if (failedPCA > 0)
		{
			std::cout << "Warning: " << failedPCA << " points resulted in default/failed normals.\n";
		}

		std::cout << "Propagating normals using Minimum Spanning Tree (MST) to ensure consistent orientation...\n";
		propagateNormalMST(filteredCloud, kdTree, K_NEIGHBOURS);
		std::cout << "Normal propagation completed.\n";

		// Print the first 5 calculated normals for verification
		std::cout << "\n--- DEBUG: First 5 Calculated Normals ---\n";
		for (size_t i = 0; i < 5 && i < filteredCloud.normals.size(); ++i)
		{
			const auto& pt = filteredCloud.points[i];
			const auto& n = filteredCloud.normals[i];
			std::cout << "Point (" << pt.x << ", " << pt.y << ", " << pt.z << ") -> "
				<< "Normal: [" << n.nx << ", " << n.ny << ", " << n.nz << "]\n";
		}

		std::cout << "Normals calculated and oriented successfully for all points.\n";

		std::string plyFilepath = "../../../data/birdfountain_normals.ply";
		std::cout << "Saving point cloud with normals to PLY...\n";
		if (PointCloudIO::saveToPLY(plyFilepath, filteredCloud))
		{
			std::cout << "PLY saved successfully to " << plyFilepath << "\n";
		}
	}
}
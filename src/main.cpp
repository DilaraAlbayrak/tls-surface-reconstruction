#include <iostream>
#include <string>
#include "PointCloud.hpp"
#include "PointCloudIO.hpp"

int main()
{
    const std::string txtFilepath = "../../../data/birdfountain_station1_xyz_intensity_rgb.txt";
    const std::string binFilepath = "../../../data/birdfountain.bin";

    PointCloud cloud;

    if (!PointCloudIO::loadOrCreateBin(txtFilepath, binFilepath, cloud))
    {
        std::cerr << "Failed to load or create binary point cloud file.\n";
        return 1;
	}

	std::cout << "\nFirst 5 points:\n";

    for (size_t i = 0; i < 5 && i < cloud.points.size(); ++i)
    {
        const auto& pt = cloud.points[i];
        std::cout << "Point " << i << ": (" << pt.x << ", " << pt.y << ", " << pt.z 
                  << "), Intensity: " << pt.intensity 
                  << ", Color: (" << pt.r << ", " << pt.g << ", " << pt.b << ")\n";
	}

	return 0;
}
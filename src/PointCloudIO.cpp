#include "PointCloudIO.hpp"
#include <iostream>
#include <fstream>

namespace
{
    constexpr size_t ESTIMATED_POINT_COUNT = 50000000;
}

namespace PointCloudIO
{
    bool loadFromBin(const std::string& filepath, PointCloud& cloud)
    {
        std::ifstream binFile(filepath, std::ios::binary);
        if (!binFile.is_open()) return false;

        binFile.seekg(0, std::ios::end);
        size_t fileSize = binFile.tellg();
        binFile.seekg(0, std::ios::beg);

        size_t pointCount = fileSize / sizeof(PointData);
        cloud.points.resize(pointCount);

        binFile.read(reinterpret_cast<char*>(cloud.points.data()), fileSize);
        return true;
    }

    bool loadFromTxt(const std::string& filepath, PointCloud& cloud)
    {
        std::ifstream txtFile(filepath);
        if (!txtFile.is_open()) return false;

        cloud.points.reserve(ESTIMATED_POINT_COUNT);

        PointData pt;
        int r_raw, g_raw, b_raw;

        while (txtFile >> pt.x >> pt.y >> pt.z >> pt.intensity >> r_raw >> g_raw >> b_raw)
        {
            pt.r = r_raw / 255.0f;
            pt.g = g_raw / 255.0f;
            pt.b = b_raw / 255.0f;
            cloud.points.push_back(pt);
        }
        return true;
    }

    bool saveToBin(const std::string& filepath, const PointCloud& cloud)
    {
        std::ofstream outBinFile(filepath, std::ios::binary);
        if (!outBinFile.is_open()) return false;

        outBinFile.write(reinterpret_cast<const char*>(cloud.points.data()), cloud.points.size() * sizeof(PointData));
        return true;
    }

    bool loadOrCreateBin(const std::string& txtFilepath, const std::string& binFilepath, PointCloud& cloud)
    {
        if (loadFromBin(binFilepath, cloud))
        {
            std::cout << "Successfully loaded " << cloud.points.size() << " points from binary file.\n";
            return true;
        }

        std::cout << "Binary file not found. Reading from text file...\n";
        if (!loadFromTxt(txtFilepath, cloud))
        {
            std::cerr << "Error: Could not open text file.\n";
            return false;
        }

        std::cout << "Successfully parsed " << cloud.points.size() << " points. Saving binary cache...\n";

        if (saveToBin(binFilepath, cloud))
        {
            std::cout << "Binary file saved successfully.\n";
        }
        else
        {
            std::cerr << "Warning: Could not save binary cache.\n";
        }

        return true;
    }

    bool saveToPLY(const std::string& filepath, const PointCloud& cloud)
    {
        std::ofstream file(filepath);
        if (!file.is_open()) return false;

        // Write PLY header
        file << "ply\n";
        file << "format ascii 1.0\n";
        file << "element vertex " << cloud.points.size() << "\n";
        file << "property float x\n";
        file << "property float y\n";
        file << "property float z\n";
        file << "property float nx\n";
        file << "property float ny\n";
        file << "property float nz\n";
        file << "property uchar red\n";
        file << "property uchar green\n";
        file << "property uchar blue\n";
        file << "end_header\n";

        // Write points, normals, and colors
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            const auto& pt = cloud.points[i];
            const auto& n = cloud.normals[i];

            // Convert RGB from [0.0, 1.0] float to [0, 255] integer
            int r = static_cast<int>(pt.r * 255.0f);
            int g = static_cast<int>(pt.g * 255.0f);
            int b = static_cast<int>(pt.b * 255.0f);

            file << pt.x << " " << pt.y << " " << pt.z << " "
                << n.nx << " " << n.ny << " " << n.nz << " "
                << r << " " << g << " " << b << "\n";
        }
        return true;
    }
}
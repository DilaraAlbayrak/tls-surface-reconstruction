#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct PointData
{
	float x, y, z;
	int intensity;
	float r, g, b;
};

constexpr size_t ESTIMATED_POINT_COUNT = 50000000;

int main()
{
	const std::string txtFilepath = "../../../data/birdfountain_station1_xyz_intensity_rgb.txt";
	const std::string binFilepath = "../../../data/birdfountain.bin";
	

	std::vector<PointData> pointCloud;

	std::ifstream binFile(binFilepath, std::ios::binary);

	if (binFile.is_open())
	{
		std::cout << "Binary file found. Loading data...\n";

		binFile.seekg(0, std::ios::end);
		size_t fileSize = binFile.tellg();
		binFile.seekg(0, std::ios::beg);

		size_t pointCount = fileSize / sizeof(PointData);
		pointCloud.resize(pointCount);

		binFile.read(reinterpret_cast<char*>(pointCloud.data()), fileSize);
		binFile.close();

		std::cout << "Successfully loaded " << pointCloud.size() << " points from binary.\n";
	}
	else
	{
		std::cout << "Binary file not found. Reading text file and converting...\n";

		std::ifstream txtFile(txtFilepath);
		if (!txtFile.is_open())
		{
			std::cerr << "Error: Could not open text file at " << txtFilepath << "\n";
			return 1;
		}
		
		/*std::string line;
		while (std::getline(txtFile, line))
		{
			std::istringstream iss(line);

			PointData pt;
			int r_raw, g_raw, b_raw;

			if (iss >> pt.x >> pt.y >> pt.z >> pt.intensity >> r_raw >> g_raw >> b_raw)
			{
				pt.r = r_raw / 255.0f;
				pt.g = g_raw / 255.0f;
				pt.b = b_raw / 255.0f;

				pointCloud.push_back(pt);
			}
		}*/

		// Pre-allocate memory for ~40 million points to prevent RAM reallocations
		pointCloud.reserve(ESTIMATED_POINT_COUNT);

		PointData pt;
		int r_raw, g_raw, b_raw;

		// Read directly from the file stream instead of parsing strings line by line
		while (txtFile >> pt.x >> pt.y >> pt.z >> pt.intensity >> r_raw >> g_raw >> b_raw) {
			pt.r = r_raw / 255.0f;
			pt.g = g_raw / 255.0f;
			pt.b = b_raw / 255.0f;
			pointCloud.push_back(pt);
		}

		txtFile.close();

		std::cout << "Successfully parsed " << pointCloud.size() << " points from text.\n";
		std::cout << "Saving to binary format for future use...\n";

		std::ofstream outBinFile(binFilepath, std::ios::binary);
		if (outBinFile.is_open())
		{
			outBinFile.write(reinterpret_cast<const char*>(pointCloud.data()), pointCloud.size() * sizeof(PointData));
			outBinFile.close();
			std::cout << "Binary file saved successfully.\n";
		}
		else
		{
			std::cerr << "Error: Could not create binary file at " << binFilepath << "\n";
		}
	}

	std::cout << "\nFirst 5 points:\n";
	for (size_t i = 0; i < 5 && i < pointCloud.size(); ++i) {
		std::cout << "X: " << pointCloud[i].x << " Y: " << pointCloud[i].y << " Z: " << pointCloud[i].z << "\n";
	}

	return 0;
}
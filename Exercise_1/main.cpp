#include <iostream>
#include <fstream>

#include "Eigen.h"

#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = 0;
	nVertices = width * height;

	// TODO: Get number of faces
	unsigned nFaces = (width * height) - 2;




	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for (int i = 0; i < nVertices; i++)
	{
		if ((vertices[i].position[0] != MINF) && (vertices[i].position[1] != MINF) && (vertices[i].position[2] != MINF)) {
			outFile << vertices[i].position[0] << " " << vertices[i].position[1] << " " << vertices[i].position[2] << " " << (int)vertices[i].color[0] << " " << (int)vertices[i].color[1] << " " << (int)vertices[i].color[2] << " " << (int)vertices[i].color[3] << std::endl;

		}
		else {
			outFile << "0 0 0 "<< (int)vertices[i].color[0] << " " << (int)vertices[i].color[1] << " " << (int)vertices[i].color[2] << " " << (int)vertices[i].color[3] << std::endl;

		}
	}





	// TODO: save faces
	for (int i = 0; i < nVertices; i++)
	{
		float currentEdgeLength_1 = 0;
		float currentEdgeLength_2 = 0;
		float currentEdgeLength_3 = 0;
		/*float edgeLength = sqrt(pow(vertices[i].position[0] - vertices[i + 1].position[0], 2)
								+ pow(vertices[i].position[1] - vertices[i + 1].position[1], 2)
								+ pow(vertices[i].position[2] - vertices[i + 1].position[2], 2));*/

		if ((i + 1) % width == 0) {
			continue;
		}

		if (i < (nVertices - width)) {
			currentEdgeLength_1 = sqrt(pow(vertices[i].position[0] - vertices[i + 1].position[0], 2)
									+ pow(vertices[i].position[1] - vertices[i + 1].position[1], 2)
									+ pow(vertices[i].position[2] - vertices[i + 1].position[2], 2));

			currentEdgeLength_2 = sqrt(pow(vertices[i].position[0] - vertices[i + width].position[0], 2)
									+ pow(vertices[i].position[1] - vertices[i + width].position[1], 2)
									+ pow(vertices[i].position[2] - vertices[i + width].position[2], 2));

			currentEdgeLength_3 = sqrt(pow(vertices[i + width].position[0] - vertices[i + 1].position[0], 2)
									+ pow(vertices[i + width].position[1] - vertices[i + 1].position[1], 2)
									+ pow(vertices[i + width].position[2] - vertices[i + 1].position[2], 2));

			if ((currentEdgeLength_1 < edgeThreshold) && (currentEdgeLength_2 < edgeThreshold) && (currentEdgeLength_3 < edgeThreshold)) {
				outFile << "3 " << i << " " << (i + 1) << " " << (i + width) << std::endl;
			}

			
		}

		if (i >= (width)) {

			currentEdgeLength_1 = sqrt(pow(vertices[i].position[0] - vertices[i + 1].position[0], 2)
				+ pow(vertices[i].position[1] - vertices[i + 1].position[1], 2)
				+ pow(vertices[i].position[2] - vertices[i + 1].position[2], 2));


			currentEdgeLength_2 = sqrt(pow(vertices[i].position[0] - vertices[i - width + 1].position[0], 2)
				+ pow(vertices[i].position[1] - vertices[i - width + 1].position[1], 2)
				+ pow(vertices[i].position[2] - vertices[i - width + 1].position[2], 2));

			currentEdgeLength_3 = sqrt(pow(vertices[i - width + 1].position[0] - vertices[i + 1].position[0], 2)
				+ pow(vertices[i - width + 1].position[1] - vertices[i + 1].position[1], 2)
				+ pow(vertices[i - width + 1].position[2] - vertices[i + 1].position[2], 2));

			if ((currentEdgeLength_1 < edgeThreshold) && (currentEdgeLength_2 < edgeThreshold) && (currentEdgeLength_3 < edgeThreshold)) {
				outFile << "3 " << i << " " << (i - width + 1) << " " << (i + 1) << std::endl;
			}

			
		}

		// outFile << "3 " << vertices[i].position[0] << " " << vertices[i].position[1] << " " << vertices[i].position[2] << " " << (int)vertices[i].color[0] << " " << (int)vertices[i].color[1] << " " << (int)vertices[i].color[2] << std::endl;
	}





	// close file
	outFile.close();

	return true;
}

int main()
{
	std::string filenameIn = "./data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	int idx = 0;
	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		
		for (int i = 0; i < sensor.GetDepthImageHeight() * sensor.GetDepthImageWidth(); i++) {
			if (depthMap[i] == MINF)
			{
				vertices[i].position = Vector4f(MINF, MINF, MINF, MINF);
				vertices[i].color = Vector4uc(0, 0, 0, 0);
			}
			else
			{
				float rowMatrix = i / sensor.GetDepthImageWidth();
				float colMatrix = i % sensor.GetDepthImageWidth();

				float x = (rowMatrix - cX) * depthMap[i] / fovX;
				float y = (colMatrix - cY) * depthMap[i] / fovY;

				Vector4f camera = Vector4f(x, y, depthMap[i], 1);
				vertices[i].position = trajectoryInv * depthExtrinsicsInv * camera;
				vertices[i].color = Vector4uc(colorMap[4 * i], colorMap[4 * i + 1], colorMap[4 * i + 2], colorMap[4 * i + 3]);
			}
		}





		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;

		idx++;
	}

	return 0;
}

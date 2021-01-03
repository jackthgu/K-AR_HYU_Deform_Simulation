#pragma once

#include <iostream>
#include <fstream>
#include <istream>
#include <string>
#include <algorithm>

#include <opencv2/opencv.hpp>

class Settings {
private:
	int m_frameWidth;
	int m_frameHeight;
	float m_zFar;
	float m_zNear;
	std::string m_intrinsicFile;
	cv::Mat m_intrinsic;

	std::string m_initPoseFile;
	cv::Mat m_initialPose = cv::Mat();

	std::string m_modelFile;
	std::string m_modelPath;

	std::string m_imageExtension;
public:
	int GetFrameWidth() { return m_frameWidth; }

	void SetFrameWidth(int val) { m_frameWidth = val; }


	int GetFrameHeight() { return m_frameHeight; }

	void SetFrameHeight(int val) { m_frameHeight = val; }


	float GetZFar() { return m_zFar; }

	void SetZFar(float val) { m_zFar = val; }


	float GetZNear() { return m_zNear; }

	void SetZNear(float val) { m_zNear = val; }


	cv::Mat GetIntrinsic() { return m_intrinsic; }

	void SetIntrinsic(cv::Mat intr) { m_intrinsic = intr; }


	cv::Mat GetInitPose() { return m_initialPose; }

	void SetInitPose(cv::Mat pose) { m_initialPose = pose; }


	std::string GetModelFile() { return m_modelFile; }

	std::string GetModelPath() { return m_modelPath; }

	std::string GetImageExtension() { return m_imageExtension; }


	void LoadSetting(const char *fileName) {
		std::ifstream settingFile(fileName);
		if (settingFile.is_open()) {
			std::string line;
			while (std::getline(settingFile, line)) {
				line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
				if (line[0] == '#' || line.empty())
					continue;

				auto delimiterPos = line.find("=");
				auto name = line.substr(0, delimiterPos);
				auto value = line.substr(delimiterPos + 1);

				std::cout << name << ":" << value << std::endl;
				try {
					if (name.compare("FrameWidth") == 0) {
						SetFrameWidth(std::stoi(value));
					} else if (name.compare("FrameHeight") == 0) {
						SetFrameHeight(std::stoi(value));
					} else if (name.compare("InitialPoseFile") == 0) {
						m_initPoseFile = value;
						cv::FileStorage fs(m_initPoseFile, cv::FileStorage::READ);
						fs.getFirstTopLevelNode() >> m_initialPose;
						m_initialPose.convertTo(m_initialPose, CV_32F);
						std::cout << fs.getFirstTopLevelNode().name() << ":" << std::endl;
						for (int i = 0; i < m_initialPose.rows; i++) {
							for (int j = 0; j < m_initialPose.cols; j++) {
								std::cout << m_initialPose.at<float>(i, j) << "\t";
							}
							std::cout << std::endl;
						}
					} else if (name.compare("ModelPath") == 0) {
						m_modelPath = value;
					} else if (name.compare("ModelFile") == 0) {
						m_modelFile = value;
					} else if (name.compare("zFar") == 0) {
						SetZFar(std::stof(value));
					} else if (name.compare("zNear") == 0) {
						SetZNear(std::stof(value));
					} else if (name.compare("IntrinsicFile") == 0) {
						m_intrinsicFile = value;
						cv::FileStorage fs(m_intrinsicFile, cv::FileStorage::READ);
						fs.getFirstTopLevelNode() >> m_intrinsic;
						m_intrinsic.convertTo(m_intrinsic, CV_32F);
						std::cout << fs.getFirstTopLevelNode().name() << ":" << std::endl;
						for (int i = 0; i < m_intrinsic.rows; i++) {
							for (int j = 0; j < m_intrinsic.cols; j++) {
								std::cout << m_intrinsic.at<float>(i, j) << "\t";
							}
							std::cout << std::endl;
						}
					} else if(name.compare("RecordImageExtension") == 0)
					{
						m_imageExtension = value;
					}
					std::cout << "Success!" << std::endl;
				}
				catch (...) {
					std::cout << "Failed.. " << std::endl;
				}


			}
		} else {
			std::cerr << "Couldn't open config file for reading[" << fileName << "]\n";
		}

	}

	void SaveSetting(const char *fileName) {
		std::ofstream os(fileName);
//		FrameWidth=1280
//		FrameHeight=720
//		InitialPoseFile=../data/initPose.yml
//		zFar=1000.0
//		zNear=0.1
//		IntrinsicFile=../data/cam_calib.yml
//		ModelFile=tri100.obj
//		ModelPath=../data/
//					 RecordImageExtension=bmp
		os << "FrameWidth=" << m_frameWidth << std::endl
		<< "FrameHeight=" << m_frameHeight << std::endl
		<< "InitialPoseFile=" << m_initPoseFile << std::endl
		<< "IntrisicFile=" << m_intrinsicFile << std::endl
		<< "zFar=" << m_zFar << std::endl
		<< "zNear=" << m_zNear << std::endl
		<< "ModelFile=" << m_modelFile << std::endl
		<< "ModelPath=" << m_modelPath << std::endl
		<< "RecordImageExtension=" << m_imageExtension << std::endl;

		cv::FileStorage fs_pose(m_initPoseFile, cv::FileStorage::WRITE);
		fs_pose.write("pose", m_initialPose);

		cv::FileStorage fs_intrisic(m_intrinsicFile, cv::FileStorage::WRITE);
		fs_intrisic.write("intrinsic", m_intrinsic);
	}
};
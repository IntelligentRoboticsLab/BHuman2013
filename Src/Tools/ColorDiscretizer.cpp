#include "ColorDiscretizer.h"

#include <fstream>
#include <cmath>


ColorDiscretizer::ColorDiscretizer(unsigned clusterNum) : clusterNum_(clusterNum), isClustered_(false) {
}


void ColorDiscretizer::setClusters(unsigned clusterNum) {
		clusterNum_ = clusterNum;
		reset();
}


bool ColorDiscretizer::initializeColorModel(const std::vector<Image::Pixel> &pixels) {
	  if (!checkClusters(clusterNum_)) return false;

    int sampleCount = 0;
    cv::Mat samples(pixels.size(), 3, CV_32F);
    for (unsigned i = 0; i < pixels.size(); ++i) {
        samples.at<float>(sampleCount, 0) = pixels[i].y;
        samples.at<float>(sampleCount, 1) = pixels[i].cb;
        samples.at<float>(sampleCount, 2) = pixels[i].cr;
        ++sampleCount;
    }

    generateModel(samples);
    return true;
}


std::vector<int> ColorDiscretizer::discretize(const std::vector<Image::Pixel> & pixels) {
    if (!isClustered()) return std::vector<int>(0);

    std::vector<int> discretizedPixels(pixels.size());
    for (unsigned i = 0; i < pixels.size(); ++i) {
        auto & pixel = pixels[i];
        discretizedPixels[i] = getColorClass(pixel.y, pixel.cb, pixel.cr);
    }
    return discretizedPixels;
}

void ColorDiscretizer::generateModel(const cv::Mat &samples) {
#if defined(MODEL_TYPE_KMEANS)
	  cv::TermCriteria criteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 5, 1.0);
	  int attempts = 3;
	  int flags = cv::KMEANS_PP_CENTERS;
    cv::Mat labels;

    cv::kmeans(samples, clusterNum_, labels, criteria, attempts, flags, clusterColors_);
#elif defined(MODEL_TYPE_EM)
		model_ = cv::EM(clusterNum_);
		model_.train(samples);
#endif
}


unsigned ColorDiscretizer::getColorClass(float channel1, float channel2, float channel3) {
#if defined(MODEL_TYPE_KMEANS)
    float minDist = INFINITY, dist;
    unsigned minIndex = clusterNum_;
    cv::Vec3f *row;

    for (unsigned i = 0; i < clusterNum_; ++i) {
        row = clusterColors_.ptr<cv::Vec3f>(i);
        dist = (channel1 - (*row)[0]) * (channel1 - (*row)[0]);
        dist += (channel2 - (*row)[1]) * (channel2 - (*row)[1]);
        dist += (channel3 - (*row)[2]) * (channel3 - (*row)[2]);

        if (dist < minDist)
        {
            minDist = dist;
            minIndex = i;
        }
    }
    return minIndex;
#elif defined(MODEL_TYPE_EM)
    cv::Mat sample(1, 3, CV_32F), probs(1, clusterNum_, CV_64FC1);
		cv::Point maxLocation;
		sample.at<float>(0, 0) = channel1;
		sample.at<float>(0, 1) = channel2;
		sample.at<float>(0, 2) = channel3;
		model_.predict(sample, probs);
		cv::minMaxLoc(probs, NULL, NULL, NULL, &maxLocation);
		if (maxLocation.x < maxLocation.y) {
			std::cout << "x" << std::endl;
			return maxLocation.x;
		}
		else {
			std::cout << "y" << std::endl;
			return maxLocation.y;
		}
#endif
}


void ColorDiscretizer::saveClusters(const std::string & fileName) {
    if (!isClustered()) return;

#if defined(MODEL_TYPE_KMEANS)
    std::ofstream outBinFile;
    outBinFile.open(fileName, std::ios::out | std::ios::binary);
    for (int i = 0; i < getClusterNum(); i++){
        outBinFile.write(reinterpret_cast<char*> (clusterColors_.ptr<cv::Vec3f>(i)), sizeof(cv::Vec3f));
    }
    outBinFile.close();
#elif defined(MODEL_TYPE_EM)
		// TODO
#endif
}


void ColorDiscretizer::readClusters(const std::string & fileName) {
#if defined(MODEL_TYPE_KMEANS)
    std::ifstream inBinFile;
    inBinFile.open(fileName, std::ios::in | std::ios::binary);
    for (int i = 0; i < getClusterNum(); i++){
        inBinFile.read(reinterpret_cast<char*> (clusterColors_.ptr<cv::Vec3f>(i)), sizeof(cv::Vec3f));
    }
    inBinFile.close();
#elif defined(MODEL_TYPE_EM)
		// TODO
#endif
}

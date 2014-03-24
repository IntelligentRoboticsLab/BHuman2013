#include "ColorDiscretizer.h"

#include <fstream>
#include <cmath>


ColorDiscretizer::ColorDiscretizer(unsigned clusterNum) : clusterNum_(clusterNum) {
}


void ColorDiscretizer::setClusters(unsigned clusterNum) {
    clusterNum_ = clusterNum;
}


bool ColorDiscretizer::initializeColorModel(const std::vector<Image::Pixel> &pixels, int clusters) {
    if (!checkClusters(clusters)) return false;

    int sampleCount = 0;
    cv::Mat samples(pixels.size(), 3, CV_32F);
    for (unsigned i = 0; i < pixels.size(); ++i) {
        samples.at<float>(sampleCount, 0) = pixels[i].y;
        samples.at<float>(sampleCount, 1) = pixels[i].cb;
        samples.at<float>(sampleCount, 2) = pixels[i].cr;
        sampleCount++;
    }

    generateClusterIndex(samples);
    return true;
}


std::vector<int> ColorDiscretizer::discretize(const std::vector<Image::Pixel> & pixels) {
    if (!isClustersIndexed_) return std::vector<int>(0);

    std::vector<int> discretizedPixels(pixels.size());
    for (unsigned i = 0; i < pixels.size(); ++i) {
        auto & pixel = pixels[i];
        discretizedPixels[i] = getColorClass(pixel.y, pixel.cb, pixel.cr);
    }
    return discretizedPixels;
}

void ColorDiscretizer::generateClusterIndex(const cv::Mat &samples,
                                            const cv::TermCriteria &criteria,
                                            int attempts,
                                            int flags) {
    cv::Mat labels;
    cv::kmeans(samples, clusterNum_, labels, criteria, attempts, flags, clusterColors_);
}


unsigned ColorDiscretizer::getColorClass(float channel1, float channel2, float channel3) {
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
}


void ColorDiscretizer::saveClusters(const std::string & fileName) {
    if (!isClustersIndexed_) return;

    std::ofstream outBinFile;
    outBinFile.open(fileName, std::ios::out | std::ios::binary);
    for (int i = 0; i < getClusterNum(); i++){
        outBinFile.write(reinterpret_cast<char*> (clusterColors_.ptr<cv::Vec3f>(i)), sizeof(cv::Vec3f));
    }
    outBinFile.close();
    return;
}


void ColorDiscretizer::readClusters(const std::string & fileName) {
    std::ifstream inBinFile;
    inBinFile.open(fileName, std::ios::in | std::ios::binary);
    for (int i = 0; i < getClusterNum(); i++){
        inBinFile.read(reinterpret_cast<char*> (clusterColors_.ptr<cv::Vec3f>(i)), sizeof(cv::Vec3f));
    }
    inBinFile.close();
    return;
}

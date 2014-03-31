#pragma once

#include <opencv2/core/core.hpp>

#include "Representations/Infrastructure/Image.h"

class ColorDiscretizer
{
    public:
        ColorDiscretizer(unsigned clusterNum);
        void setClusters(unsigned clusterNum);
        bool initializeColorModel(const std::vector<Image::Pixel> & pixels);
        std::vector<int> discretize(const std::vector<Image::Pixel> & pixels);
        unsigned getColorClass(float channel1, float channel2, float channel3);

        void saveClusters(const std::string & filename);
        void readClusters(const std::string & filename);

        inline bool isClustered() const { return isClustersIndexed_; }
        inline int getClusterNum() const { return clusterNum_; }
        inline cv::Mat getClusterColors() const { return clusterColors_; }

    private:
        unsigned clusterNum_;
        cv::Mat clusterColors_;
        bool isClustersIndexed_;
        inline bool checkClusters(int clusterNum) { return (clusterNum > 0) && (clusterNum_ = clusterNum); }
        void generateClusterIndex(const cv::Mat & samples,
                                  const cv::TermCriteria & criteria=cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 5, 1.0),
                                  int attempts=3,
                                  int flags=cv::KMEANS_PP_CENTERS);
};

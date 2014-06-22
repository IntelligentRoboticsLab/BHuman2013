#pragma once

#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#include "Representations/Infrastructure/Image.h"

//#define MODEL_TYPE_KMEANS
#define MODEL_TYPE_EM 
#define OLD_OPENCV CV_MAJOR_VERSION < 2 || (CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION <= 3)
#define NEW_OPENCV !OLD_OPENCV

class ColorDiscretizer
{
    public:
        ColorDiscretizer(unsigned clusterNum);
        void setClusters(unsigned clusterNum);
        bool initializeColorModel(const std::vector<Image::Pixel> & pixels);
        std::vector<int> discretize(const std::vector<Image::Pixel> & pixels) const;
        unsigned getColorClass(float channel1, float channel2, float channel3) const; // Model-specific

        void saveClusters(const std::string & filename); // Model-specific
        void readClusters(const std::string & filename); // Model-specific

        inline bool isClustered() const {
#if defined(MODEL_TYPE_KMEANS)
					return isClustered_;
#elif defined(MODEL_TYPE_EM)
#if OLD_OPENCV
          return isTrained_;
#else
					return model_.isTrained();
#endif
#endif
				}
        inline void reset() {
#if defined(MODEL_TYPE_KMEANS)
					isClustered_ = false;
#elif defined(MODEL_TYPE_EM)
#if OLD_OPENCV
          //model_ = cv::CvEM(emParams_)
#else
					model_ = cv::EM(clusterNum_);
#endif
#endif
				}
        inline int getClusterNum() const { return clusterNum_; }
        inline cv::Mat getClusterColors() const { return clusterColors_; }

    private:
        unsigned clusterNum_;
        cv::Mat clusterColors_;
#if defined(MODEL_TYPE_KMEANS)
        bool isClustered_;
#elif defined(MODEL_TYPE_EM)
#if OLD_OPENCV
        bool isTrained_;
        cv::CvEM model_;
#else
				cv::EM model_;
#endif
#endif
        inline bool checkClusters(int clusterNum) { return (clusterNum > 0) && (clusterNum_ = clusterNum); }
        void generateModel(const cv::Mat & samples); // Model-specific
};

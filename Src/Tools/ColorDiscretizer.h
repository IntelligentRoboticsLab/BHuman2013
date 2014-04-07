#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#include "Representations/Infrastructure/Image.h"

//#define MODEL_TYPE_KMEANS
#define MODEL_TYPE_EM 

class ColorDiscretizer
{
    public:
        ColorDiscretizer(unsigned clusterNum);
        void setClusters(unsigned clusterNum);
        bool initializeColorModel(const std::vector<Image::Pixel> & pixels);
<<<<<<< HEAD
        std::vector<int> discretize(const std::vector<Image::Pixel> & pixels);
        unsigned getColorClass(float channel1, float channel2, float channel3); // Model-specific
=======
        std::vector<int> discretize(const std::vector<Image::Pixel> & pixels) const;
        unsigned getColorClass(float channel1, float channel2, float channel3) const;
>>>>>>> VisualCompass nearly compiles.

        void saveClusters(const std::string & filename); // Model-specific
        void readClusters(const std::string & filename); // Model-specific

        inline bool isClustered() const {
#if defined(MODEL_TYPE_KMEANS)
					return isClustered_;
#elif defined(MODEL_TYPE_EM)
					return model_.isTrained();
#endif
				}
        inline void reset() {
#if defined(MODEL_TYPE_KMEANS)
					isClustered_ = false;
#elif defined(MODEL_TYPE_EM)
					model_ = cv::EM(clusterNum_);
#endif
				}
        inline int getClusterNum() const { return clusterNum_; }
        inline cv::Mat getClusterColors() const { return clusterColors_; }

    private:
        unsigned clusterNum_;
        cv::Mat clusterColors_;
#if defined(MODEL_TYPE_EM
				cv::EM model_;
#endif
        bool isClustered_;
        inline bool checkClusters(int clusterNum) { return (clusterNum > 0) && (clusterNum_ = clusterNum); }
        void generateModel(const cv::Mat & samples); // Model-specific
};

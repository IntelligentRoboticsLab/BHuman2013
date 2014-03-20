#ifndef COLORDISCRETIZER_H
#define COLORDISCRETIZER_H

#include <opencv2/core/core.hpp>
#include "Tools/ImageProcessing/ImagePrimitives.h"
#include "Representations/Infrastructure/Image.h"
#include <fstream>
#include <stdio.h>
#include <string>

class ColorDiscretizer
{
public:
    ColorDiscretizer();
    ~ColorDiscretizer();
    /*
    */
    void setClusters(int num_clusters);
    // TODO: fix the cluster number arguments
    bool initializeColorModel(cv::InputArrayOfArrays images, int clusters=-1);
    bool initializeColorModel(const std::vector<naoth::Image> &images, int clusters=-1);
    bool initializeColorModel(const std::vector<Pixel> &pixels, int clusters=-1);
    void discretize(cv::InputArray image, cv::OutputArray discretizedImage);
    void discretize(const naoth::Image &image, std::vector<std::vector<int> > &discretizedImage);
    void discretize(const std::vector<Pixel> &pixels, std::vector<int> &discretizedPixels);
    inline int getClusters() const { return this->clusters; }
    inline cv::Mat getClusterColors() const { return this->clusterColors; }
    void saveClusters(const char* fileName);
    void readClusters(const char* fileName);
    unsigned int nearestNeighborIndex(float channel1, float channel2, float channel3);
private:
    unsigned int clusters;
    cv::Mat clusterColors;
    bool isClustersIndexed;
    inline bool checkClusters(int clusters) { return (clusters > 0) && (this->clusters = clusters); }
    void generateClusterIndex(const cv::Mat &samples,
                              const cv::TermCriteria &criteria=cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 5, 1.0),
                              int attempts=3,
                              int flags=cv::KMEANS_PP_CENTERS);

};

#endif // COLORDISCRETIZER_H

#pragma once

#include <vector>
#include <chrono>

#include "Representation/Infrastructure/Image.h"
//include for ColorDiscretizer

template <typename T>
using Table3D = std::vector<std::vector<std::vector<T>>>;

template <typename T>
using Table2D = std::vector<std::vector<T>>>;

class VisualCompassFeature {
    public:
        using FeatureTableType = Table3D<double>;
        
        VisualCompassFeature(const VisualCompassParameters & params);
        void initFromScanlines(const Table2D<Pixel> & scanline, const ColorDiscretizer & clusterer);
        
        double compare(const VisualCompassFeature &) const;
        double getCertainty(/* possibly unsigned timestamp */) const; // See how to pass, using frameinfo style

    private:
        const VisualCompassParameters & params_;
        FeatureTabletype featureTable_;
        //int length_;
        double agentOrientation_;
        // std::chrono::time_point time_;
        // Vector2<double> source_position;
        double measurementCertainty_;
        bool isValid_;
};

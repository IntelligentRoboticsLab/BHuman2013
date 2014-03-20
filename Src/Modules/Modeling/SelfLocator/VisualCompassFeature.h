#pragma once

#include <vector>
#include <chrono>
#include <iosfwd>

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

        void makeValid(const VisualCompassFeature &, double newOrientation, double newCertainty);

        void initFromScanlines(const Table2D<Pixel> & scanline, const ColorDiscretizer & clusterer);
        
        double compare(const VisualCompassFeature &) const;

        const VisualCompassParameters & getParams() const;
        const FeatureTabletype & getFeatureTable() const;
        double getAgentOrientation() const;
        double getCertainty(/* possibly unsigned timestamp */) const; // See how to pass, using frameinfo style


        bool isValid() const;

    private:
        const VisualCompassParameters & params_;
        FeatureTabletype featureTable_;
        //int length_;
        double agentOrientation_;
        // std::chrono::time_point time_;
        // Vector2<double> source_position;
        double measurementCertainty_;
        bool isValid_;

        friend std::istream & operator>>(std::istream & os, VisualCompassFeature & vcf);
};

// Be careful that these two depend on the currently set parameters! If you save a Feature,
// change the parameters and then reload it it's going to load up the wrong number of things
// and it's going to badly break (possibly not even telling you). BE WARNED.
std::ostream & operator<<(std::ostream & os, const VisualCompassFeature & vcf);
std::istream & operator>>(std::istream & os, VisualCompassFeature & vcf);

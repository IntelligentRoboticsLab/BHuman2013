#pragma once

#include <iosfwd>

#include "VisualCompassTypes.h"
#include "Representations/Infrastructure/Image.h"
#include "VisualCompassParameters.h"
#include "Tools/ColorDiscretizer.h"

class VisualCompassFeature {
    public:
        using FeatureTableType = Table3D<double>;
        
        VisualCompassFeature(const VisualCompassParameters & params);

        void makeValid(const VisualCompassFeature &, double newOrientation, double newCertainty);
        void makeInvalid();

        void initFromScanlines(const Table2D<Image::Pixel> & scanline, const ColorDiscretizer & clusterer);
        
        double compare(const VisualCompassFeature &) const;

        const VisualCompassParameters & getParams() const;
        const FeatureTableType & getFeatureTable() const;
        double getAgentOrientation() const;
        double getCertainty(/* possibly unsigned timestamp */) const; // See how to pass, using frameinfo style


        bool isValid() const;

    private:
        const VisualCompassParameters * params_;
        FeatureTableType featureTable_;

        double agentOrientation_;
        double measurementCertainty_;
        bool isValid_;

        friend std::istream & operator>>(std::istream & os, VisualCompassFeature & vcf);
};

// Be careful that these two depend on the currently set parameters! If you save a Feature,
// change the parameters and then reload it it's going to load up the wrong number of things
// and it's going to badly break (possibly not even telling you). BE WARNED.
std::ostream & operator<<(std::ostream & os, const VisualCompassFeature & vcf);
std::istream & operator>>(std::istream & os, VisualCompassFeature & vcf);

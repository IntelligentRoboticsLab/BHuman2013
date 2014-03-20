#include "VisualCompassFeature.h"

VisualCompassFeature::VisualCompassFeature(const VisualCompassParameters & params) : params_(params), isValid_(false)
{
    featureTable_.resize(params_.compassFeatureNum);
    for (unsigned i = 0; i < params_.compassFeatureNum; ++i) {
        featureTable_[i].resize(params_.colorNum);
        for (unsigned j = 0; j < params_.colorNum; ++j)
            featureTable_[i][j].resize(params_.colorNum, 0.0);
    }
}

double VisualCompassFeature::getCertainty(unsigned time) const 
{
		// TODO: how does time work?
    return measurementCertainty_; // * exp(someTimeValueInSecs(difftime(time, current_time))
}

double VisualCompassFeature::compare(const VisualCompassFeature & other) const
{
    double similarityMeasure = 0.0;
    for (unsigned index = 0; index < params_.compassFeatureNum; ++index)
        for (unsigned i = 0; i < params_.colorsNum; ++i)
            for (unsigned j = 0; j < params_.colorsNum; ++j)
                similarityMeasure += std::abs((1 - featureTable_[index][i][j]) - (1 - other.featureTable_[index][i][j]));
    return similarityMeasure;
}

void VisualCompassFeature::initFromScanlines(const Table2D<Pixel> & scanlines, const ColorDiscretizer & clusterer)
{
    if (scanlines.size() == 0) return;
    
    for (unsigned stripe = 0; stripe < scanlines.size(); ++stripe)
    {
        auto labels = clusterer.discretize(scanlines[stripe]));
        unsigned size = labels.size();
        for (unsigned i = 1; i < size; ++i)
            featureTable_[stripe][labels[i-1]][labels[i]] += 1.0 / static_cast<double>(size);
    }    
}

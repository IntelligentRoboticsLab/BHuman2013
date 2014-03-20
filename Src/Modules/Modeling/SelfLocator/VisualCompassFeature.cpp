#include "VisualCompassFeature.h"
#include <iostream>

VisualCompassFeature::VisualCompassFeature(const VisualCompassParameters & params) : params_(params), isValid_(false)
{
    featureTable_.resize(params_.compassFeatureNum);
    for (unsigned i = 0; i < params_.compassFeatureNum; ++i) {
        featureTable_[i].resize(params_.colorNum);
        for (unsigned j = 0; j < params_.colorNum; ++j)
            featureTable_[i][j].resize(params_.colorNum, 0.0);
    }
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

void VisualCompassFeature::makeValid( const VisualCompassFeature & other, double newOrientation, double newCertainty ) {
    featureTable_ = other.featureTable_;
    agentOrientation_ = newOrientation;
    measurementCertainty_ = newCertainty;

    // TODO: FIXME: Shouldn't we be updating the time here?
     
    isValid_ = true;
}

const VisualCompassParameters & VisualCompassFeature::getParams() const {
    return params_;
}

const VisualCompassFeature::FeatureTableType & VisualCompassFeature::getFeatureTable() const {
    return featureTable_;
}

double VisualCompassFeature::getAgentOrientation() const {
    return agentOrientation_;
}

double VisualCompassFeature::getCertainty(/* unsigned time */) const 
{
	// TODO: how does time work?
    return measurementCertainty_; // * exp(someTimeValueInSecs(difftime(time, current_time))
}

bool VisualCompassFeature::isValid() const {
    return isValid_;
}

std::ostream & operator<<(std::ostream & os, const VisualCompassFeature & vcf) {
    for (unsigned index = 0; index < vcf.getParams().compassFeatureNum; ++index)
        for (unsigned i = 0; i < vcf.getParams().colorsNum; ++i)
            for (unsigned j = 0; j < vcf.getParams().colorsNum; ++j)
                os << vcf.getFeatureTable()[index][i][j];
    os << vcf.getAgentOrientation();
    os << vcf.getCertainty();
    os << vcf.isValid();

    return os;
}

std::istream & operator>>(std::istream & is, VisualCompassFeature & vcf) {
    for (unsigned index = 0; index < vcf.getParams().compassFeatureNum; ++index)
        for (unsigned i = 0; i < vcf.getParams().colorsNum; ++i)
            for (unsigned j = 0; j < vcf.getParams().colorsNum; ++j)
                is >> vcf.featureTable_[index][i][j];
    is >> vcf.agentOrientation_;
    is >> vcf.measurementCertainty_;
    is >> vcf.isValid_;

    return is;
}

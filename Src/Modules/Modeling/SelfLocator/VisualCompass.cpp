#include "VisualCompass.h"

// For debugging
#include "Tools/Debugging/Debugging.h"

MAKE_MODULE(VisualCompass, Perception)

VisualCompass::VisualCompass() : grid_(parameters, theFieldDimensions), discretizer_(parameters.colorNum), frameSkip_(0) {
}

void VisualCompass::update(VisualPole&) {
    updateHorizon();

    OUTPUT(idText, text, "ViCTOriA here!");

    grid_.resetConfidence();
    grid_.updateConfidence(theSampleSet);

    // Here be many DEBUG_REQUESTs
    //
    frameSkip_ = (frameSkip_ + 1) % parameters.frameSkipNum;
}

void VisualCompass::victoria() {
    if ( !validHorizon() || !discretizer_.isClustered() ) return;

    VisualCompassFeature feature;
    feature.initFromScanlines(verticalScanner(), discretizer_);

    auto bestMatches = grid_.bestMatches(feature, theRobotPose);

    double confidence = DBL_MAX;
    double orientation = 0.0;
    for(unsigned a = 0; a < bestMatches.size(); ++a) {
        if( bestMatches[a].y < confidence ) {
            confidence = bestMatches[a].y;
            orientation = bestMatches[a].x;
        }
    }

    /*
    if ( bestMatches.size() && orientation != theRobotPose.rotation ) {
        If you want to compute error statistics you can do it here.
    }
    */
}

/*
 * These were like statistics counters that we are not using.
 * If you'd like them, implement them yourself. Ah. Ah.
void VisualCompass::resetCounters()
{
    total = 0;
    has_answer = 0;
    sum_angle_error = 0;
    square_sum_angle_error = 0;
}
*/

void VisualCompass::reset() {
    grid_.resetConfidence();
    grid_.resetFeatures();

    discretizer_.reset();
    dataPixels_.clear();
}

void VisualCompass::clusterColors() {
    if (!dataPixels_.empty() && !discretizer_.isClustered())
        discretizer_.initializeColorModel(dataPixels_);
}

Table2D<Image::Pixel> VisualCompass::verticalScanner() {
    std::vector<std::vector<Image::Pixel>> result;
    result.reserve(N-1);

    double N = parameters.compassFeatureNum;
    // N-.5 is because we want to avoid double approximation errors.
    for ( double x = 0.0; x < (N-.5); ++x ) {
        auto horizonPoint = ((x + .5)/N)*leftHorizon_ + ((N-x-.5)/N)*rightHorizon_;
        auto horizonCoordinatedPoint = theImageCoordinateSystem.toHorizonBased(horizonPoint);

        std::vector<Image::Pixel> line;
        auto imageCoordinatedPoint = theImageCoordinateSystem.fromHorizonBased(horizonCoordinatedPoint);
        while ( inImage(imageCoordinatedPoint) ) {
            line.push_back(theImage[imageCoordinatedPoint.y][imageCoordinatedPoint.x]);
            horizonCoordinatedPoint.y += 1;
            imageCoordinatedPoint = theImageCoordinateSystem.fromHorizonBased(horizonCoordinatedPoint);
        }
        result.push_back(line);
    }
    return result;
}

bool VisualCompass::validHorizon() {
    static const double maxArea = theImage.width * theImage.height;
    double currentArea = theImage.wdith * (leftHorizon_.y + rightHorizon_.y) / 2.0;
    double areaCovered = currentArea / maxArea;

    return (areaCovered >= parameters.compassAreaMinRatio &&
            areaCovered <= parameters.compassAreaMaxRatio);
}


void VisualCompass::updateClusterPixels() {
    if ( frameSkip_ || discretizer_.isClustered() ) return;

    for ( unsigned x = 0; x < theImage.width; ++x )
        for ( unsigned y = 0; y < theImage.height; ++y )
            dataPixels_.push_back(theImage[y][x]);
}

void VisualCompass::recordFeatures() {
    if ( !validHorizon() || !discretizer_.isClustered() ) return;

    VisualCompassFeature feature;
    feature.initFromScanlines(verticalScanner(), discretizer_);

    grid_.storeFeature(feature, theRobotPose);
}

void VisualCompass::updateHorizon() {
    static const Vector2<int> bottomLeft(0, theImage.height);
    static const Vector2<int> topRight(theImage.width, 0);
    Geometry::Line horizon = Geometry::calculateHorizon(theCameraMatrix, theCameraInfo);
    Geometry::getIntersectionPointsOfLineAndRectangle(bottomLeft, topRight, horizon, leftHorizon_, rightHorizon_);
}

bool VisualCompass::inImage(const Vector2<>& imageCoordPoint) const {
    return  imageCoordPoint.x >= 0.0 && imageCoordPoint.x <= theImage.width &&
            imageCoordPoint.y >= 0.0 && imageCoordPoint.y <= theImage.height;
}

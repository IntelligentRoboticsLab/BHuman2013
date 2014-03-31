#include "VisualCompass.h"

// For debugging
#include "Tools/Debugging/Debugging.h"

MAKE_MODULE(VisualCompass, Perception)

VisualCompass::VisualCompass() : grid_(parameters, theFieldDimensions), discretizer_(parameters.colorNum) {
}

void VisualCompass::update(VisualPole&) {
    OUTPUT(idText, text, "ViCTOriA here!");
    grid_.resetConfidence();
    grid_.updateConfidence(theSampleSet);
 
    // Here be many DEBUG_REQUESTs
}


/*
void VisualCompass::resetCounters()
{
    total = 0;
    has_answer = 0;
    sum_angle_error = 0;
    square_sum_angle_error = 0;
}
*/


void VisualCompass::clear() {
  grid_ = VisualCompassGrid(parameters, theFieldDimensions);
  discretizer_ = ColorDiscretizer(parameters.colorNum);
  pixelsForClustering_.clear();
}


void VisualCompass::clusterColors() {
  if (!pixelsForClustering_.empty() && !discretizer_.isClustered()) {
    discretizer_.initializeColorModel(pixelsForClustering_);
  }
}


std::vector<std::vector<Image::Pixel>> VisualCompass::verticalScanner() {
}


bool VisualCompass::validHorizon() {
  static const Vector2<int> bottomLeft(0, theImage.height);
  static const Vector2<int> topRight(theImage.width, 0);

  Geometry::Line horizon = Geometry::calculateHorizon(theCameraMatrix, theCameraInfo);

  Geometry::getIntersectionPointsOfLineAndRectangle(bottomLeft, topRight, horizon, leftHorizon_, rightHorizon_);

  static const double maxArea = theImage.width * theImage.height;
  double currentArea = theImage.wdith * (leftHorizon_.y + rightHorizon_.y) / 2.0;
  double areaCovered = currentArea / maxArea;

  return (areaCovered >= parameters.compassAreaMinRatio &&
      areaCovered <= parameters.compassAreaMaxRatio);
}


void VisualCompass::colorExtraction() {
}


void VisualCompass::recordFeatures() {
}


void VisualCompass::victoria() {
}

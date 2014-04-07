#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"

#include "Representations/Perception/VisualPole.h"

#include "Tools/ColorDiscretizer.h"
#include "Tools/Math/Geometry.h"

#include <vector>

MODULE(VisualCompass)
    REQUIRES(Image)
    REQUIRES(CameraMatrix)
    REQUIRES(CameraInfo)
    REQUIRES(ImageCoordinateSystem)
    REQUIRES(FieldDimensions)
    REQUIRES(FrameInfo)
    REQUIRES(RobotPose)

    PROVIDES(VisualPole)

    LOADS_PARAMETERS(VisualCompassParameters, parameters)
END_MODULE

template <typename T>
using Table2D = std::vector<std::vector<T>>>;

class VisualCompass : public VisualCompassBase
{
private:
    VisualCompassGrid grid_;
    ColorDiscretizer discretizer_;
    std::vector<Image::Pixel> dataPixels_;
    Vector2<int> leftHorizon_, rightHorizon_;

    void update(VisualPole&);
    void reset();
    void clusterColors();
    bool validHorizon();
    Table2D<Image::Pixel> verticalScanner();

    // Not yet implemented but necessary
    void colorExtraction();
    void recordFeatures(); // sort of integrate it into something
    void victoria();

    void updateHorizon();
    bool inImage(const Vector2<>& imageCoordPoint) const;

    unsigned frameSkip_;
};

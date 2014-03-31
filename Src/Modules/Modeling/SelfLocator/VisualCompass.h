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

class VisualCompass : public VisualCompassBase
{
private:
    VisualCompassGrid grid_;
    ColorDiscretizer discretizer_;
    std::vector<Image::Pixel> pixelsForClustering_;
    Vector2<int> leftHorizon_, rightHorizon_;

    void update(VisualPole&);
    void clear();
    void clusterColors();
    bool validHorizon();

    // Not yet implemented but necessary
    std::vector<std::vector<Image::Pixel>> verticalScanner();
    void colorExtraction();
    void recordFeatures(); // sort of integrate it into something
    void victoria();
};


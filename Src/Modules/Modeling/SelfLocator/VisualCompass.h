#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/UKFSamples.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"

#include "Representations/Perception/VisualPole.h"

#include "Tools/ColorDiscretizer.h"
#include "Tools/Math/Geometry.h"

#include "VisualCompassParameters.h"
#include "VisualCompassGrid.h"
#include "VisualCompassTypes.h"

#include <vector>

MODULE(VisualCompass)
    REQUIRES(Image)
    USES(CameraMatrix)
    REQUIRES(CameraInfo)
    USES(ImageCoordinateSystem)
    REQUIRES(FieldDimensions)
    REQUIRES(FrameInfo)
    REQUIRES(RobotPose)
    REQUIRES(UKFSamples)

    PROVIDES(VisualPole)

    LOADS_PARAMETER(VisualCompassParameters, parameters)
END_MODULE

class VisualCompass : public VisualCompassBase
{
public:
    VisualCompass();
private:
    VisualCompassGrid grid_;
    ColorDiscretizer discretizer_;
    std::vector<Image::Pixel> dataPixels_;
    Vector2<int> leftHorizon_, rightHorizon_;

    void updateClusterPixels();
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
    bool inImage(const Vector2<int>& imageCoordPoint) const;

    unsigned frameSkip_;
};


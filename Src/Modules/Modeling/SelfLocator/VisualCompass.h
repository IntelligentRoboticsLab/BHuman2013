#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImageCoordinateSystem.h"

#include "Representations/Perception/VisualPole.h"

MODULE(VisualCompass)
    REQUIRES(Image)
    REQUIRES(ImageCoordinateSystem)
    REQUIRES(FieldDimensions)
    REQUIRES(FrameInfo)
    REQUIRES(RobotPose)

    PROVIDES(VisualPole)

    // Parameters for the VisualCompass
    LOADS_PARAMETER(float, compassWidthMin)
    LOADS_PARAMETER(float, compassWidthMax)
    LOADS_PARAMETER(int, compassFeatureNumber)
//    LOADS_PARAMETER(std::string, compassDataFile)
    LOADS_PARAMETER(bool, compassParticleUpdate)

    // Parameters for ColorDiscretizer
    LOADS_PARAMETER(int, colorNum)
//    LOADS_PARAMETER(std::string, clusterDataFile)

    // Parameters for VisualCompassFeature
    LOADS_PARAMETER(int, compassFeatureNum)

    // Parameters for VisualGridMapProvider
    LOADS_PARAMETER(int, gridXLength)
    LOADS_PARAMETER(int, gridYLength)
    LOADS_PARAMETER(int, angleBinsNum)
    LOADS_PARAMETER(float, angleSize)

    // Parameters for WeightedExperts
    LOADS_PARAMETER(float, smoothingFactor)
    LOADS_PARAMETER(float, gridCellConfidence)
END_MODULE

class VisualCompass : public VisualCompassBase
{
private:
    void update(VisualPole&);
};


#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"

#include "Representations/Perception/VisualPole.h"

MODULE(VisualCompass)
    REQUIRES(Image)
    REQUIRES(ImageCoordinateSystem)
    REQUIRES(FieldDimensions)
    REQUIRES(RobotPose)

    PROVIDES(VisualPole)

    DEFINES_PARAMETER(int, myParamTest, 42)
END_MODULE

class VisualCompass : public VisualCompassBase
{
private:
    void update(VisualPole&);
};


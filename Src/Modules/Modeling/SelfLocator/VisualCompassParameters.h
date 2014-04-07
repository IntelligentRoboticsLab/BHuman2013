#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(VisualCompassParameters,
{,
    (float) compassWidthMin,
    (float) compassWidthMax,
    (unsigned) compassFeatureNumber,
//    (std::string) compassDataFile,
    (bool) compassParticleUpdate,
    (unsigned) frameSkipNum,

    // Parameters for ColorDiscretizer
    (unsigned) colorsNum,
//    (std::string) clusterDataFile,

    // Parameters for VisualCompassFeature
    (unsigned) compassFeatureNum,

    // Parameters for VisualGridMapProvider
    (unsigned) gridXLength,
    (unsigned) gridYLength,
    (unsigned) angleBinsNum,
    (float) angleSize,

    // Parameters for WeightedExperts
    (float) smoothingFactor,
    (float) gridCellConfidence,
    (float) compassAreaMinRatio,
    (float) compassAreaMaxRatio,
});

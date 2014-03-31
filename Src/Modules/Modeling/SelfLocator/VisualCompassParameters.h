#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(VisualCompassParameters,
{,
    (float) compassWidthMin,
    (float) compassWidthMax,
    (int) compassFeatureNumber,
//    (std::string) compassDataFile,
    (bool) compassParticleUpdate,

    // Parameters for ColorDiscretizer
    (int) colorNum,
//    (std::string) clusterDataFile,

    // Parameters for VisualCompassFeature
    (int) compassFeatureNum,

    // Parameters for VisualGridMapProvider
    (int) gridXLength,
    (int) gridYLength,
    (int) angleBinsNum,
    (float) angleSize,

    // Parameters for WeightedExperts
    (float) smoothingFactor,
    (float) gridCellConfidence,
});

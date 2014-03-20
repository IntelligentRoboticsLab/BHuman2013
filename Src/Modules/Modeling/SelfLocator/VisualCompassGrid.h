#pragma once

#include <vector>
//#include "VisualCompassParameters.h"
#include "VisualCompassFeature.h"
#include "Representations/Infrastructure/FieldInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include <time.h>

class VisualCompassParameters;
class FieldDimensions;
class UKFSample;
class Vector3f;
class RobotPose;

template <typename T> class Vector2;
template <typename T> class Vector3;
template <typename T> class SampleSet;

template <typename T>
using Table3D = std::vector<std::vector<std::vector<T>>>;

template <typename T>
using Table2D = std::vector<std::vector<T>>;

class VisualCompassGrid {
    public:
        using ConfidenceGridType = Table2D<float>;
        using FeatureGridType = Table3D<VisualCompassFeature>;

        VisualCompassGrid(const VisualCompassParameters & params, const FieldDimensions & info);

        Vector2<int> fieldPosToGridPos(float x, float y) const;
        int fieldPosToGridPos(float angle) const;

        void updateConfidence(SampleSet<UKFSample>* samples);
        void resetConfidence();

        void storeFeature(const VisualCompassFeature & vcf, const Pose2D & robotPose);

        // Debug Only
        // YOU CANNOT CHANGE THE PARAMETERS IN BETWEEN A SAVE
        // AND A READ, SINCE THEY ARE INTEGRAL PART OF THE SIZE
        // OF WHAT GETS SAVED (tables). BE CAREFUL SINCE LOADING MAY
        // FAIL SILENTLY (for example if you increased table sizes).
        void saveGridMapModel() const;
        void readGridMapModel();

    private:
        const VisualCompassParameters & params_;
        const FieldDimensions & fieldInfo_;

        ConfidenceGridType confidenceGrid_;
        FeatureGridType    featureGrid_;

        double cellXLength_, cellYLength_;
};

#include "VisualCompassGrid.h"

#include "Tools/Math/Common.h"
#include "Tools/SampleSet.h"
#include "UKFSample.h"

#include <algorithm>

VisualCompassGrid::VisualCompassGrid(const VisualCompassParameters & params, const FieldDimensions & info) : params_(params), fieldInfo_(info)
{
    featureGrid_.resize(params_.gridXLength);
    confidenceGrid_.resize(params_.gridXLength);
    for (unsigned i = 0; i < params_.gridXLength; ++i) {
        featureGrid_[i].resize(params_.gridYLength);
        confidenceGrid_[i].resize(params_.gridYLength, 0.0);
        for (unsigned j = 0; j < params_.gridYLength; ++j)
            featureGrid_[i][j].resize(params_.angleBinsNum, params_);
    }

    cellXLength_ = ( 2 * fieldInfo_.xPosOpponentGroundline) / params_.gridXLength;
    cellYLength_ = ( 2 * fieldInfo_.yPosLeftSideline )      / params_.gridYLength;
}

Vector2<int> VisualCompassGrid::fieldPosToGridPos(float x, float y) const {
    // Initial pose starts centered in field center. We translate it into a corner.
    //find the proper bin and grid cell
    double posX = x + fieldInfo_.xPosOpponentGroundline; // Half field length
    double posY = y + fieldInfo_.yPosLeftSideline; // Half field width

    Vector2<int> result;

    result.x = std::max(0, (int) floor(posX / cellXLength_));
    result.x = std::min(params_.gridXLength - 1, static_cast<unsigned>(x));

    result.y = std::max(0, (int) floor(posY / cellYLength_));
    result.y = std::min(params_.gridYLength - 1, static_cast<unsigned>(y));

    return result;
}

int VisualCompassGrid::fieldAngleToGridAngle(float angle) const {
    // angle is between -pi to +pi
    double theta = toDegrees(angle) + 180.0; // From Math/Common.h
    // Finds the angleBin index we were looking for
    theta = theta * params_.angleBinsNum / 360.0;

    return static_cast<int>(theta);
}

void VisualCompassGrid::resetConfidence() {
  for (unsigned i = 0; i < params_.gridXLength; ++i)
      for (unsigned j = 0; j < params_.gridYLength; ++j)
        confidenceGrid_[i][j] = 0.0;
}

void VisualCompassGrid::resetFeatures() {
    for (unsigned i = 0; i < params_.gridXLength; i++)
        for (unsigned j = 0; j < params_.gridYLength; j++)
            for (unsigned k = 0; k < params_.angleBinsNum; k++)
                featureGrid_[i][j][k].makeInvalid();
}

void VisualCompassGrid::updateConfidence(const SampleSet<UKFSample>& samples) {
    int size = samples.size();
    for (int i = 0; i < size; i++) {
        const UKFSample & sample = samples.at(i);
        auto gridPos = fieldPosToGridPos(sample.mean.x, sample.mean.y);
        confidenceGrid_[gridPos.x][gridPos.y] += 1.0 / static_cast<double>(size);
    }
}

// takes a vector with features and store them in the grid
void VisualCompassGrid::storeFeature(const VisualCompassFeature & vcf, const Pose2D & robotPose) {
    int gridAngle = fieldAngleToGridAngle(robotPose.rotation);
    // leave it 0 if you want to have a dynamic update based on the mc particles.
    if ( params_.compassParticleUpdate ) {
        for (unsigned x = 0; x < params_.gridXLength; ++x) {
            for (unsigned y = 0; y < params_.gridYLength; ++y) {
                if ( confidenceGrid_[x][y] > 0.00) {
                    auto & feature = featureGrid_[x][y][gridAngle];
                    // Pick invalid Feature and fill it with new data
                    if( ! feature.isValid() || feature.getCertainty() <= confidenceGrid_[x][y] ) {
                        feature.makeValid(vcf, robotPose.rotation, confidenceGrid_[x][y]);
                    }
                }
            }
        }
    }
    else {
        auto gridPos = fieldPosToGridPos(robotPose.translation.x, robotPose.translation.y);
        auto & feature = featureGrid_[gridPos.x][gridPos.y][gridAngle];
        if( ! feature.isValid() || feature.getCertainty() <= confidenceGrid_[gridPos.x][gridPos.y] ) {
            feature.makeValid(vcf, robotPose.rotation, confidenceGrid_[gridPos.x][gridPos.y]);
        }
    }
}

/*
 * These need the compassDataFile parameter
void VisualCompassGrid::saveGridMapModel() const {
    std::ofstream outBinFile(params_.compassDataFile, ios::out | ios::binary);
    for (unsigned i = 0; i < params_.gridXLength; i++)
        for (unsigned j = 0; j < params_.gridYLength; j++)
            for (unsigned k = 0; k < params_.angleBinsNum; k++)
                outBinFile << featureGrid_[i][j][k];
}

void VisualCompassGrid::readGridMapModel() {
    std::ifstream inBinFile(params_.compassDataFile, ios::in | ios::binary);
    for (unsigned i = 0; i < params_.gridXLength; i++)
        for (unsigned j = 0; j < params_.gridYLength; j++)
            for (unsigned k = 0; k < params_.angleBinsNum; k++)
                inBinFile >> featureGrid_[i][j][k];
}
*/

std::vector<Vector2<double>> VisualCompassGrid::bestMatches(const VisualCompassFeature & inputFeature, const Pose2D & robotPose) {
    auto gridPos = fieldPosToGridPos(robotPose.translation.x, robotPose.translation.y);
    std::vector<Vector2<double>> output;

    if(params_.compassParticleUpdate) {
        std::vector<Vector2<int>> cells;
        for (unsigned x = 0; x < params_.gridXLength; ++x)
            for (unsigned y = 0; y < params_.gridYLength; ++y)
                if(confidenceGrid_[x][y] > 0.20)
                    cells.push_back(Vector2<int>(x, y));
        if( !cells.empty() ) {
            for(unsigned a = 0; a < params_.angleBinsNum; ++a) {
                double similaritySum = 0.00;
                for(unsigned i = 0; i < cells.size(); ++i) {
                    auto & cell = cells[i];
                    auto & feature = featureGrid_[cell.x][cell.y][a];
                    if ( feature.isValid() ) {
                        double similarity = feature.compare(inputFeature);
                        similarity *= 1 - confidenceGrid_[cell.x][cell.y];
                        similaritySum += similarity;
                    }
                }
                // Old computation.. ??
                // output.emplace_back(a * (params_.angleBinsNum / 360.0), sum_similarity);
                output.emplace_back(360.0 * a / params_.angleBinsNum, similaritySum);
            }
        }
        return output;
    }
    else {
        double min_sim = DBL_MAX;
        double orientation = 0.0;
        for(unsigned a = 0; a < params_.angleBinsNum; ++a) {
            auto & feature = featureGrid_[gridPos.x][gridPos.y][a];
            if( feature.isValid() ) {
                double similarity = feature.compare(inputFeature);
                // multiplied by 1-conf, cause we have min similarity, less means better.
                similarity *= 1 - confidenceGrid_[gridPos.x][gridPos.y];
                if( similarity < min_sim ) {
                    min_sim = similarity;
                    orientation = feature.getAgentOrientation();
                }
            }
        }
        output.emplace_back(orientation, min_sim);
    }
    return output;
}

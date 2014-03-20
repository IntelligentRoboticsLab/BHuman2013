#include "VisualCompassGrid.h"

VisualCompassGrid::VisualCompassGrid(const VisualCompassParameters & params, const FieldDimensions & info) : params_(params), fieldInfo_(info)
{
    featureGrid_.resize(params_.xGrid);
    confidenceGrid_.resize(params_.xGrid);
    for (unsigned i = 0; i < params_.xGrid; ++i) {
        featureTable_[i].resize(params_.yGrid);
        confidenceGrid_[i].resize(params_.yGrid, 0.0);
        for (unsigned j = 0; j < yGrid_; ++j)
            featureTable_[i][j].resize(params_.angleBins, fparams_);
    }

    cellXLength_ = ( 2 * fieldInfo_.xPosOpponentGroundLine) / params_.xGrid;
    cellYLength_ = ( 2 * fieldInfo_.yPosLeftSideLine )      / params_.yGrig;
}

Vector2<int> VisualCompassGrid::fieldPosToGridPos(float x, float y) {
    // Initial pose starts centered in field center. We translate it into a corner.
    //find the proper bin and grid cell
    double posX = x + fieldInfo_.xPosOpponentGroundLine; // Half field length
    double posY = y + fieldInfo_.yPosLeftSideLine; // Half field width

    Vector2<int> result;

    result.x = max(0, (int) floor(posX / cellXLength_));
    result.x = min(params_.xGrid - 1, x);

    result.y = max(0, (int) floor(posY / cellYLength_));
    result.y = min(params_.yGrid - 1, y);

    return result;
}

int VisualCompassGrid::fieldAngletoGridAngle(float angle) {
    // angle is between -pi to +pi
    double theta = Math::toDegrees(angle) + 180.0;
    // Finds the angleBin index we were looking for
    theta = theta * params_.angleBins / 360.0;

    return theta;
}

void VisualCompassGrid::resetConfidence() {
  for (unsigned i = 0; i < params_.xGrid; ++i)
      for (unsigned j = 0; j < params_.yGrid; ++j)
        confidenceGrid_[i][j] = 0.0;
}

void VisualCompassGrid::updateConfidence(SampleSet<UKFSample>* samples) {
    int size = samples->size();
    for (int i = 0; i < size; i++) {
        UKFSample & sample = samples->at(i);
        auto gridPos = fieldPosToGridPos(sample.mean.x, sample.mean.y);
        confidengeGrid_[gridPos.x][gridPos.y] += 1.0 / static_cast<double>(size);
    }
}

// takes a vector with features and store them in the grid
void storeFeature(const VisualCompassFeature & vcf, const Pose2D & robotPose) {
    if ( ! robotPose.isValid ) return;

    int gridAngle = fieldAngletoGridAngle(robotPose.rotation);
    // leave it 0 if you want to have a dynamic update based on the mc particles.
    if ( params_.compassParticleUpdate ) {
        for (int x = 0; x < params_.xGrid; ++x) {
            for (int y = 0; y < params_.yGrid; ++y) {
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
        if( ! feature.isValid() || feature.getCertainty() <= confidenceGrid_[x][y] ) {
            feature.makeValid(vcf, robotPose.rotation, confidenceGrid_[x][y]);
        }
    }
}

void VisualCompassGrid::saveGridMapModel() {
    std::ofstream outBinFile(params_.compassDataFile, ios::out | ios::binary);
    for (int i = 0; i < params_.gridX; i++)
        for (int j = 0; j < params_.gridY; j++)
            for (int k = 0; k < params_.angleBins; k++)
                outBinFile << featureGrid_[i][j][k];
}

void readGridMapModel() {
    std::ifstream inBinFile(params_.compassDataFile, ios::in | ios::binary);
    for (int i = 0; i < params_.gridX; i++)
        for (int j = 0; j < params_.gridY; j++)
            for (int k = 0; k < params_.angleBins; k++)
                inBinFile >> featureGrid_[i][j][k];
}

/**
* @file VisualCompass.h
*
* @author <a href="mailto:giorgosmethe@gmail.com">Methenitis, Georgios</a>
* @author <a href="mailto:dhrdekok@gmail.com">de Kok, Patrick</a>
* Definition of class VisualCompass
*/

#ifndef VISUALCOMPASS_H
#define VISUALCOMPASS_H

// others
#include <ModuleFramework/Module.h>
#include <vector>
#include <string>

// tools
#include "Tools/Math/Line.h"
#include "Tools/ColorClasses.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Matrix_nxn.h"
#include "Tools/Math/Matrix_mxn.h"
#include "Tools/Math/Geometry.h"
#include "Tools/ImageProcessing/BresenhamLineScan.h"
#include "Tools/ImageProcessing/ColoredGrid.h"

// infrastructure
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/ColorTable64.h"
#include "Representations/Infrastructure/CameraSettings.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FieldInfo.h"

// perception
#include "Representations/Perception/FieldPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ArtificialHorizon.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/ObjectPercept.h"
#include "Cognition/Modules/Modeling/SelfLocator/MonteCarloSelfLocator/MonteCarloSelfLocator.h"

// modelling
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ColorClassificationModel.h"
#include "Representations/Modeling/AttentionModel.h"

// motion
#include "Representations/Motion/Request/HeadMotionRequest.h"
#include "Representations/Motion/Request/MotionRequest.h"
#include "Representations/Motion/MotionStatus.h"

//debug requests
#include "Tools/Debug/DebugRequest.h"
#include "Tools/Debug/DebugModify.h"
#include "DebugCommunication/DebugCommandManager.h"

// visual compass stuff
#include "Cognition/Modules/Perception/VisualCompass/VisualCompassParameters.h"
#include "Cognition/Modules/Perception/VisualCompass/VisualCompassFeature.h"
#include "Cognition/Modules/Perception/VisualCompass/VisualGridMapProvider.h"
#include "Cognition/Modules/Perception/VisualCortex/ColorDiscretizer.h"
#include "Cognition/Modules/Perception/VisualCompass/WeightedExperts.h"

BEGIN_DECLARE_MODULE(VisualCompass)
/* Used for camera images. Used interface:
 * - ::get(x, y) -> Pixel {uchar channels[3];}
 * - ::width() -> NUMBER
 * - ::height() -> NUMBER
 * - ::isInside(int point_x, int point_y) -> bool
 *       Tests if a pixel (point_x, point_y) is inside the image
 * Defined in BHuman: /Src/Representations/Infrastructure/Image.h
 * Expected porting issues: none.
 */
REQUIRE(Image)

// Unused!
//REQUIRE(ColorClassificationModel)
// Unused!
//REQUIRE(CurrentCameraSettings)
// Unused!
//REQUIRE(CameraMatrix)

/* Interface:
 * - ::begin() -> Vector2<double>: begin point of horizon line
 * - ::end() -> Vector2<double>: end point of horizon line
 * - ::point(double p) -> Vector2<double>: the point <x,y> on the horizon with x = p
 * - ::getDirection() -> {rotateRight() -> {angle() -> double}}
 * Defined in BHuman: /Src/Representations/Perception/ImageCoordinateSystem.h
 * Expected porting issues: rewrite code that used ArtificialHorizon. Not that much.
 */
REQUIRE(ArtificialHorizon)

// Unused!
//REQUIRE(FieldPercept)
// Unused!
//REQUIRE(ColoredGrid)

/* Used to compute in which grid cell the robot is, and to draw stuff in DebugMode to the screen.
 * Interface:
 * - double ::xLength: length of field lines
 * - double ::yLength: length of field lines
 * Defined in BHuman: /Config/Locations/Default/fieldDimensions.cfg 
 * Defined in BHuman: /Src/Representations/Configuration/FieldDimensions.h
 * Expected porting issues: Renaming of variables. 
 * xLength = 2 * xPosOpponentGroundline
 * yLength = 2 * yPosLeftSideline
 */
REQUIRE(FieldInfo) 

// Unused!
//REQUIRE(OpenCVGrayScale)

/* Interface:
 * - ::getFrameNumber() -> [int, uint]: how many-th image it is, so you can select only the n-th image to process
 * Solution: count how often the procedure is called.  If it has been called n times, reset counter and execute.  Otherwise, no-op.
 */
REQUIRE(FrameInfo)

// Unused!
//REQUIRE(MotionStatus)

/* Seems to be the particle filter for localisation? Also seems to be a SampleSet, defined in NaoTH2011-light/NaoTHSoccer/Source/Core/Cognition/Modules/Modeling/SelfLocator/MonteCarloSelfLocator/SampleSet.{h,cpp}
 * Interface:
 * - SampleSet ::theSampleSet
 * - double ::rotation: orientation on the field
 * - {double x, double y} ::translation: position on the field, wrt a certain corner
 * Defined in BHuman: /Src/Representations/Modeling/RobotPose.h
 * Expected porting issues: Not sure on BHuman's particle filter method.  We need more info on that, and see if ViCTOriA is compatible with it as is.
 */
REQUIRE(RobotPose)

/* Only used to send specific requests to MotionRequest.
 * - enum HeadMotionID ::look_straight_ahead
 * Defined in BHuman: /Src/Representations/MotionControl/HeadMotionRequest.h
 * Expected porting issues: Completely different motion system. Just say how the head should be in joint angles.  Not 100% necessary as it's only in debug code.
 */
PROVIDE(HeadMotionRequest)

/* Used for moving in certain poses
 * Used interface:
 * - bool ::forced: not sure, set to false
 * - {int, float, double} ::standHeight: to make it sit, it's set to -1
 * Defined in BHuman: /Src/Representations/MotionControl/MotionRequest.h
 * Expected porting issues: Different system to pass on motion requests.  Not 100% necessary as it's only in debug code.
 */
PROVIDE(MotionRequest)

END_DECLARE_MODULE(VisualCompass)

// This file has been touched by the documentation effort

/**
 * @brief This class provides a visual compass which updates during usage.
 *
 * The implementation is based on "ViCTOriA: Visual Compass To Orientate 
 * Accurately" by de Kok et al.[1] and "Orientation finding using a grid based
 * visual compass" by Methenitis et al.[2]
 *
 * [1] See for the project report: 
 *   http://www.dutchnaoteam.nl/wp-content/uploads/2013/07/ViCTOriA.pdf
 *
 * [2] See for the publication:
 *   http://www.science.uva.nl/~arnoud/publications/GridBasedVisualCompass.pdf
 */
class VisualCompass: private VisualCompassBase
{
public:
    VisualCompass();
    ~VisualCompass();
    void execute();
private:
		/**
		 * @brief Number of images used for making the color profile. 
		 * Only used for debug info.
		 */
    int num_images;
    /**
     * @brief Collection of all pixels of the images used for making a color model.
     */
    vector<Pixel> pixelVector;
    VisualGridMapProvider GridMapProvider;
    ColorDiscretizer ClusteringProvider;
    WeightedExperts queryModel;
    void drawClusteredColors();
    void drawInfo();

    int total, has_answer;
    double sum_angle_error, square_sum_angle_error;

    /**
     * @brief This function sets all counters back to 0.
     */
    void resetCounters();

    /**
     * @brief This function stores current color clusters to file.
     */
    void saveColorClusters();

    /**
     * @brief This function retrieves color clusters from file.
     */
    void readColorClusters();

    /**
     *
     */
    void staticColorClusters();

    /*
     *
     */
    void head();

    /*
     *
     */
    void motion();

    /*
     *
     */
    bool clustered;

    /*
     *
     */
    void initializeGridMapModel();

    /*
     *
     */
    void extractPixelsFromImages();

    /*
     *
     */
    void drawPoseOrientation();

    /*
     *
     */
    void drawCompassOrientation(double orientation);

    /*
     *
     */
    void drawROI();

    /*
     *
     */
    void colorClustering();

    /*
     *
     */
    void drawVisualGridModel();

    /*
     *
     */
    void drawCellConfidence();

    /*
     *
     */
    void recordedFeatures();

    /*
     *
     */
    void victoria();

    /**
     * @brief This function store features in the grid map.
     *
     * During the off line phase, this method records features
     * and creates the model. You can enable this function through
     * robotControl.
     */
    void recordFeatures();

    /**
     * @brief This function reads a saved model from the disk, if present.
     */
    void readGridMapModel();

    /**
     * @brief This function saves the current model to a binary file.
     *
     * The file is stored in the config directory.
     * We call this method after the recording function, to save the model.
     */
    void saveGridMapModel();

    /**
     * @brief This function checks if there is already a model saved to disk.
     */
    bool hasGridMapModel();

    /**
     * @brief This function clears any previous files stored from the compass. 
     *
     * You can enable this function only from RobotControl.
     */
    void clearCompass();

    /**
     * @brief This function checks if the visual compass can make an estimate.
     *
     * If the part of the image below the horizon is more than a certain 
     * threshold, the compass will be disabled to make any estimate, as too
     * little information is present to base its orientational estimate on.
     */
    bool validHorizon();

    /**
     * @brief This function stores all pixels of an image in pixelVector.
     */
    void colorExtraction();

    /**
     * @brief This function gets scanlines from an image, if allowed.
     *
     * For this, the function first retrieves an image from the camera, 
     * computes the position of the horizon in that image.  It then collects
     * lines of pixels, 10 px from each other and orthogonal to the horizon.
     * This is put in the stripes parameter.
     *
     * @param stripes A collection of scanlines orthogonal to the horizon
     */
    void verticalScanner(vector< vector<Pixel> > &stripes);

    /**
     * @brief This function lets the robot do an initialization routine.
     *
     * The routine lets the robot make a full turn at its spot, and collect
     * all features it sees.  This way, there will be a (nearly) complete 
     * model at the robot's current grid cell.
     */
    void mapping(cv::Mat &mappingImages);

    /**
     * @brief This function lets the robot sit in a stable position for recording.
     */
    void scannerPosition();


};//end class VisualCompass

#endif // VISUALCOMPASS_H

/*******************************************************************************
 *  ImageSources.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef ImageSources_H
#define ImageSources_H

#include <string>
#include <sstream>
#include <map>

#include "Mutex.h"

/**
 * @class  ImageSources
 * @brief  Holds identifiers and descriptions of different image sources
 * @author David Gossow (RX)
 */
class ImageSources
{
  public:

    /** @enum SourceId Identifies the different image sources
      *
      * @warning Don't forget to add a description for all sources in fillSourceDesc()
      * @note    Avoid changing existing constants for backwards compatibility
      */
    enum SourceId
    {
      //Cameras
      TopCamera=0,
      CenterCamera=1,
      BottomCamera=4,
      ThermalCamera=6,
      KinectCamera=7,
			
      //Video streams
      TopCameraStream=100,
      CenterCameraStream=101,
      BottomCameraStream=104,
      KinectCameraStream=105,
			
      WiiMoteStream=109,

      //Object Recognition
      ORPrimary=300,

      //Object Learning
      OLPrimary=400,
      OLSecondary=401,

      //Navigation
      OccupancyMap=500,
      OccupancyUpdateMap=501,
      ObstacleMap=502,

      SonarMap=520,

      ExplorationTransform=530,
      PathTransform=531,
      ObstacleTransform=532,
      TargetMap=533,
      CostTransform=534,

      //
      PeopleTracker=600,
      LaserPainterTest=601,
      ExplorationMap=602,

      //
      Roomba=700,

      //Display images for human robot interaction
      HRIDisplay = 800,

      // Gesture recognition debug
      GestureDebug1 = 900,
      GestureDebug2 = 901,
      GestureDebug3 = 902,
      GestureDebug4 = 903,

      TofFaceDetection = 1000,
      ImageFaceDetection = 1001,

      //Empty channel
      None=9999
    };

    /** @brief Fill the description map with values. Is called once on initialization. */
    /**        Only entries present here will appear in the GUI */
    static void fillSourceDesc()
    {
      sourceDesc[None]="None";

      sourceDesc[TopCamera]="Top Camera";
      sourceDesc[CenterCamera]="Middle Camera";
      sourceDesc[BottomCamera]="Bottom Camera";
      sourceDesc[KinectCamera]="Kinect Camera";
			
      sourceDesc[ThermalCamera]="Thermal Camera";

      sourceDesc[TopCameraStream]="Top Camera Stream";
      sourceDesc[CenterCameraStream]="Middle Camera Stream";
      sourceDesc[BottomCameraStream]="Bottom Camera Stream";
      sourceDesc[KinectCameraStream]="Kinect Camera Stream";

      sourceDesc[WiiMoteStream]="WiiMote Stream";

                        sourceDesc[ORPrimary]="Object Recognition";

      sourceDesc[OLPrimary]="Object Learning (Primary)";
      sourceDesc[OLSecondary]="Object Learning (Secondary)";

      sourceDesc[OccupancyMap]="Mapping: Occupancy Map";
      sourceDesc[OccupancyUpdateMap]="Mapping: Occupancy Update Map";
      sourceDesc[ObstacleMap]="Mapping: Obstacle Map";

      sourceDesc[SonarMap]="Mapping: Sonar Map";

      sourceDesc[ExplorationTransform]="Navigation: Exploration Transform";
      sourceDesc[PathTransform]="Navigation: Path Transform";
      sourceDesc[ObstacleTransform]="Navigation: Obstacle Transform";
      sourceDesc[TargetMap]="Navigation: Target Map";
      sourceDesc[CostTransform]="Navigation: Cost Transform";

      sourceDesc[PeopleTracker]="People Tracking";
      sourceDesc[LaserPainterTest]="LaserPainterTest";
      sourceDesc[ExplorationMap]="Exploration Map";

      sourceDesc[Roomba]="Roomba Debug";

      sourceDesc[HRIDisplay]="HRI Display (User Interaction)";

      sourceDesc[GestureDebug1]="Gesture Recognition Debug 1";
      sourceDesc[GestureDebug2]="Gesture Recognition Debug 2";
      sourceDesc[GestureDebug3]="Gesture Recognition Debug 3";
      sourceDesc[GestureDebug4]="Gesture Recognition Debug 4";

      sourceDesc[TofFaceDetection]="ToF camera: Face Detection";
      sourceDesc[ImageFaceDetection]="Left Camera: Face Detection";

    }

    /** @return Call fillSourceDesc if necessary and do some further initialization */
    static void initSourceDesc();

    /** @return IDs and descriptions of all available image sources */
    static std::map< SourceId, std::string > getSourceDesc();

    /** @return Description of a specific image source */
    static std::string getSourceDesc( SourceId id );

  private:

    /** @brief The constructor */
    ImageSources();

    /** @brief The destructor */
    ~ImageSources();

    static std::map<SourceId,std::string> sourceDesc;
    static Mutex mutex;

};


#endif

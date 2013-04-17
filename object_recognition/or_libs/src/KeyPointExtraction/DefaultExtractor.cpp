/*******************************************************************************
 *  DefaultExtractor.cpp
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  $Id: $
 *
 *******************************************************************************/

#include "DefaultExtractor.h"
#include "Architecture/Config/Config.h"
#include "ParallelSurfExtractor.h"
// #include "OrigSurfExtractor.h"

#include <ros/ros.h>

#define THIS DefaultExtractor

using namespace std;

KeyPointExtractor* THIS::createInstance()
{
  ExtractorType type = ExtractorType( Config::getInt( "KeyPointExtraction.iAlgorithm" ) );

  switch ( type )
  {
    case ExtParallelSurf:
      return new ParallelSurfExtractor();
      
/*    case ExtOrigSurf:
      return new OrigSurfExtractor();*/
      
    default:
      ROS_ERROR_STREAM( "Unknown extractor type!" );
      return new ParallelSurfExtractor();
  }
}

#undef THIS


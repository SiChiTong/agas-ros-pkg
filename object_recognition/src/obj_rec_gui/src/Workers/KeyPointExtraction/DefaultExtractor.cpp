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

// #include "Architecture/Config/Config.h" // TODO
// #include "Architecture/Tracer/Tracer.h" // TODO


#include "ParallelSurfExtractor.h"
// #include "OrigSurfExtractor.h"

#define THIS DefaultExtractor

using namespace std;

KeyPointExtractor* THIS::createInstance()
{
    ExtractorType type = ExtractorType(0); // TODO ExtractorType( Config::getInt( "KeyPointExtraction.iAlgorithm" ) );

  switch ( type )
  {
    case ExtParallelSurf:
      return new ParallelSurfExtractor();
      
/*    case ExtOrigSurf:
      return new OrigSurfExtractor();*/
      
    default:
      //TRACE_ERROR( "Unknown extractor type!" ); // TODO use ros
      return new ParallelSurfExtractor();
  }
}

#undef THIS


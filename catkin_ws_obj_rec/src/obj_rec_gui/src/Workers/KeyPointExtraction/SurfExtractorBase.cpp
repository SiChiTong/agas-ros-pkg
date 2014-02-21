/*******************************************************************************
 *  SurfExtractorBase.cpp
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#include "SurfExtractorBase.h"


#include <cmath>
#include <sstream>
#include <float.h>

// #include "Architecture/Config/Config.h" // TODO
// #include "Architecture/Tracer/Tracer.h" // TODO


#include "../../Workers/Puma2/openCvAdaptor.h"

#define THIS SurfExtractorBase

#define FULL_DEBUG


THIS::THIS( )
{
  m_IndexSize = 1; // TODO Config::getInt ( "KeyPointExtraction.Surf.iIndexSize" );
  m_SamplingStep = 1; // TODO Config::getInt ( "KeyPointExtraction.Surf.iSamplingStep" );
  m_Octaves = 1; // TODO Config::getInt ( "KeyPointExtraction.Surf.iOctaves" );
  m_BlobResponseThreshold = 1; // TODO Config::getFloat ( "KeyPointExtraction.Surf.fBlobResponseThreshold" );
  m_InitLobeSize = 1; // TODO  Config::getInt ( "KeyPointExtraction.Surf.iInitLobeSize" );
  m_RotationInvariance = 1; // TODO  Config::getBool ( "KeyPointExtraction.Surf.bRotationInvariance" );
  m_Extended = 1; // TODO Config::getBool ( "KeyPointExtraction.Surf.bExtended" );
}


THIS::~THIS()
{
}


THIS::THIS ( const SurfExtractorBase& other ) : KeyPointExtractor ( other )
{
  *this = other;
}



SurfExtractorBase& THIS::operator= ( const SurfExtractorBase & other )
{
  m_IndexSize = other.m_IndexSize;
  m_SamplingStep = other.m_SamplingStep;
  m_Octaves = other.m_Octaves;
  m_BlobResponseThreshold = other.m_BlobResponseThreshold;
  m_InitLobeSize = other.m_InitLobeSize;
  m_RotationInvariance = other.m_RotationInvariance;
  m_Extended = other.m_Extended;
  return *this;
}



void THIS::setSamplingStep ( int newValue )
{
  m_SamplingStep = newValue;
};

void THIS::setOctaves ( int newValue )
{
  m_Octaves = newValue;
};

void THIS::setBlobResponseThreshold ( double newValue )
{
  m_BlobResponseThreshold = newValue;
};

void THIS::setInitLobeSize ( int newValue )
{
  m_InitLobeSize = newValue;
};

void THIS::setRotationInvariance ( bool newValue )
{
  m_RotationInvariance = newValue;
};

void THIS::setExtended ( bool newValue )
{
  m_Extended = newValue;
};

std::string THIS::getDescription()
{
  std::ostringstream s;
  s << "SURF Parameters:" << std::endl << "----------------" << std::endl;

  s << std::endl << "m_IndexSize: " << m_IndexSize;
  s << std::endl << "m_SamplingStep: " << m_SamplingStep;
  s << std::endl << "m_Octaves: " << m_Octaves;
  s << std::endl << "m_BlobResponseThreshold: " << m_BlobResponseThreshold;
  s << std::endl << "m_InitLobeSize: " << m_InitLobeSize;
  s << std::endl << "m_RotationInvariance: " << m_RotationInvariance;
  s << std::endl << "m_Extended: " << m_Extended;

  s << std::endl << std::endl;

  return s.str();
}


#undef THIS


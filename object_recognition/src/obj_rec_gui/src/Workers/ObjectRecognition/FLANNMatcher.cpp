/*******************************************************************************
 *  FLANNMatcher.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include "FLANNMatcher.h"

// #include "Architecture/Tracer/Tracer.h" // TODO use ros instead
// #include "Architecture/Singleton/Clock.h" // TODO

#include "Workers/Math/vec2.h"

#include <assert.h>
#include <map>
#include <list>
#include <math.h>

using namespace std;

#define THIS FLANNMatcher

THIS::THIS ()
{
    m_hasIndex = false;
    m_descriptorLength = 64; // May differ, set when building an index

    getFlannParameters().target_precision = 0.95;
    getFlannParameters().log_level = FLANN_LOG_INFO;
    getFlannParameters().algorithm = KDTREE;
    getFlannParameters().checks = 32;
    getFlannParameters().trees = 8;
    getFlannParameters().branching = 32;
    getFlannParameters().iterations = 7;
    getFlannParameters().target_precision = -1;

    m_FlannModelData = 0;

    m_Log << "FLANNMatcher created\n";
}

THIS::THIS ( const FLANNMatcher& other )
{
  operator= ( other );
}

THIS& THIS::operator= ( const FLANNMatcher& other )
{
  if ( this == &other )
  {
    return *this;
  }

  m_Matches = other.m_Matches;

  //m_Log = other.m_Log;

  m_flannIndex = other.m_flannIndex;
  m_flannParams = other.m_flannParams;
  m_hasIndex = other.m_hasIndex;
  m_descriptorLength = other.m_descriptorLength;

  return *this;
}

THIS::~THIS()
{
  clearFLANNMembers();
  delete[] m_FlannModelData;
}

void THIS::createIndex( std::vector< KeyPoint >* keyPoints ){
  if(keyPoints->size()==0)
  {
    // TRACE_ERROR("Cannot create index, because there are no keypoints."); // TODO use ROS
    return;
  }
  clearFLANNMembers();
  if(keyPoints->size() != 0){
    m_descriptorLength = keyPoints->at(0).featureVector.size();
  }

  int numFeatures = keyPoints->size();
  if(m_FlannModelData!=0)
  {
    delete[] m_FlannModelData;
}
  m_FlannModelData = new float[numFeatures*m_descriptorLength];
  fillFlannDataWithDescriptors(keyPoints, m_FlannModelData);
  float speedup = 0.0;
  m_flannIndex = flann_build_index(m_FlannModelData, numFeatures, m_descriptorLength, &speedup, &m_flannParams);
  m_hasIndex = true;
}

void THIS::match (  std::vector< KeyPoint >* keyPoints, float maxDistRatio )
{
    m_Matches.clear();
    if ( ( keyPoints->size() ==0 ) || ( m_Matches.size() !=0 ) || (!m_hasIndex) )
    {
        //TRACE_ERROR("Cannot match features."); // TODO use ROS

        if ( keyPoints->size() ==0 )
          //  TRACE_ERROR("Key Points Size is 0."); // TODO use ROS
        if ( m_Matches.size() !=0 )
          //  TRACE_ERROR("Matches not 0."); // TODO use ROS
        if (!m_hasIndex)
          //  TRACE_ERROR("No Index."); // TODO use ROS

        return;
    }

    // int startTime = Clock::getInstance()->getTimestamp(); // TODO

    unsigned int numKeypoints = keyPoints->size();
    int numberOfNeighbours = 2; // Number of approximate nearest neigbours to be determined
    int* indices = new int[numberOfNeighbours*numKeypoints]; // Holds the indices of the nearest neighbours
    float* distances = new float[numberOfNeighbours*numKeypoints]; // Holds the distance to the 1st and 2nd nearest neighbour
    float* testset = new float[numKeypoints*m_descriptorLength]; // Contains the data of the descriptors to be matched
    fillFlannDataWithDescriptors(keyPoints, testset);
    flann_find_nearest_neighbors_index(m_flannIndex, testset, numKeypoints, indices, distances, numberOfNeighbours, &m_flannParams );


    // check for distance ratios
    double distanceRatio = 0.0;
    for(unsigned i = 0; i < numKeypoints; i++){
      distanceRatio = distances[i*2+0] / distances[i*2+1];    
      if(distanceRatio < maxDistRatio ){
          KeyPointMatch match={ indices[i*2+0], i,distanceRatio, 0, 0 };
        m_Matches.push_back ( match );
      }  
    }
    delete[] indices;
    delete[] distances;
    delete[] testset;
    // m_Log << "\n--- " << m_Matches.size() << " keypoints matched in first phase in " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms\n"; // TODO

//    eliminateMultipleMatches();
}


void THIS::eliminateMultipleMatches()
{
    //It is possible that more than one ipoint in First has been matched to
    //the same ipoint in Second, in this case eliminate all but the closest one

  // int startTime = Clock::getInstance()->getTimestamp(); // TODO

    //maps keypoints in Second to their closest match result
    //first: index in m_KeyPointsB
    //second: iterator in m_Matches
    map< unsigned, MatchElem > bestMatch;

    m_Log << "deleting ";

    MatchElem currentMatch=m_Matches.begin();
    while ( currentMatch != m_Matches.end() )
    {
        unsigned index2=currentMatch->index2;

        //check if a match with this keypoint in Second was found before
        map< unsigned, MatchElem >::iterator previous=bestMatch.find ( index2 );

        //this is the first match found which maps to current second index
        if ( previous == bestMatch.end() )
        {
            bestMatch[ index2 ] = currentMatch;
            currentMatch++;
            continue;
        }

        MatchElem previousMatch = previous->second;
        //a match mapping to this index in second has been found previously, and had a higher distance
        //so delete the previously found match
        if ( currentMatch->distance < previousMatch->distance )
        {
            m_Log << previousMatch->index1 << "->" << previousMatch->index2;
            m_Log << " (better:" << currentMatch->index1 << "->" << currentMatch->index2 << ")  ";
            m_Matches.erase ( previousMatch );
            bestMatch[ index2 ] = currentMatch;
            currentMatch++;
            continue;
        }
        //otherwise, the previously found best match is better than current,
        //so delete current
        m_Log << currentMatch->index1 << "->" << currentMatch->index2;
        m_Log << " (better:" << previousMatch->index1 << "->" << previousMatch->index2 << ")  ";
        currentMatch=m_Matches.erase ( currentMatch );
    }
    // m_Log << "\n--- " << m_Matches.size() << " remaining after multiple match elimination in " << ( Clock::getInstance()->getTimestamp() - startTime ) << "ms\n"; // TODO

}

string THIS::getLog()
{
  string log = m_Log.str();
  m_Log.str("");
  return log;
}



void THIS::fillFlannDataWithDescriptors(const std::vector< KeyPoint >* features, float* flannDataPtr){
  for(unsigned int i = 0; i < features->size(); i++){
    for(unsigned int j = 0; j < m_descriptorLength; j++){
      flannDataPtr[m_descriptorLength*i + j] = features->at(i).featureVector[j];
    }
  }
}

void THIS::clearFLANNMembers(){
  if(m_hasIndex){
    flann_free_index(m_flannIndex, &m_flannParams);
    m_hasIndex = false;
  }

}


#undef THIS

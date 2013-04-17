/*******************************************************************************
 *  ConnectedComponentAnalyzer.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:  
 *  $Id: ConnectedComponentAnalyzer.cpp 44313 2011-04-06 22:46:28Z agas $ 
 *******************************************************************************/
#include <cstring>

#include "ros/ros.h"

#include "ConnectedComponentAnalyzer.h"
#include "ConnectedComponent.h"
#include "ipcccf.h"

#include "Architecture/Singleton/Clock.h"

#define THIS ConnectedComponentAnalyzer

THIS::THIS()
 : m_LabelImage( 0 ), m_Width( 0 ), m_Height( 0 )
{
}

THIS::~THIS()
{
  if ( m_LabelImage )
  {
    delete[] m_LabelImage;
    m_LabelImage = 0;
  }
}

void THIS::isolateLargestSegment(unsigned char* binImage, unsigned width, unsigned height)
{
  // Generate image with the connected components
  unsigned *labelImage = new unsigned[ height * width ];
  ipcConnectedComponentFilter< unsigned char, unsigned > ccf8bit;
  ccf8bit.init( width, height );
  
  unsigned maxLabel = ccf8bit.execute( binImage, labelImage );
  
  std::vector<unsigned> numPixels;
  numPixels.assign( maxLabel+1, 0 );

  unsigned offset=0;
  unsigned y=0;
  unsigned x=0;
  unsigned realMaxLabel=0;
  
  //Extract segment sizes
  while(y<height) {
    while(x<width)
    {
      numPixels[ labelImage[offset] ]++;
      if (labelImage[offset] > realMaxLabel) { realMaxLabel=labelImage[offset]; }
      offset++;
      x++;
    }
    x=0;
    y++;
  }

  unsigned maxNumPixels=0;
  unsigned largestSegment=0;
  
  //determine largest segment
  for (unsigned i=1;i<=realMaxLabel;i++)
  {
    if( numPixels[i] > maxNumPixels )
    {
      maxNumPixels = numPixels[i];
      largestSegment = i;
    }
  }

  //clear all segments except largest one
  for (unsigned i=0; i<width*height; i++)
  {
    if ( labelImage[i] != largestSegment )
    {
      binImage[i]=0;
    }
  }
  delete[] labelImage;
}


std::vector< ConnectedComponent > THIS::segment(unsigned char* binImage, unsigned width, unsigned height, unsigned minSegmentSize, float minOccupiedArea, unsigned expandSegmentSize)
{
  if ( m_LabelImage )
  {
    delete[] m_LabelImage;
    m_LabelImage = 0;
  }
  m_LabelImage = new unsigned[ height * width ];
  m_Width = width;
  m_Height = height;
  
  ipcConnectedComponentFilter< unsigned char, unsigned > ccf8bit;
  ccf8bit.init( width, height );
  unsigned max_label = ccf8bit.execute( binImage, m_LabelImage );
  
  #ifdef CLOCK_OUTPUT
  cout << "ccf8bit: " << Clock::getInstance()->getTimestamp()-time << " ms" << endl;
  cout << "max_label: " << max_label << endl;
  time=Clock::getInstance()->getTimestamp();
  #endif
  
  return extractComponents( max_label, minSegmentSize, minOccupiedArea, expandSegmentSize );
}

std::vector< ConnectedComponent > THIS::segment(const float* floatImage, unsigned width, unsigned height, float maxDistance, 
                                                unsigned minSegmentSize, float minOccupiedArea, unsigned expandSegmentSize)
{
  if ( m_LabelImage )
  {
    delete[] m_LabelImage;
    m_LabelImage = 0;
  }
  m_LabelImage = new unsigned[ height * width ];
  m_Width = width;
  m_Height = height;
  
  ipcConnectedComponentFilterSim< float, unsigned, IntensitySimilarity<float> > ccf8bit;
  ccf8bit.init( width, height );
  unsigned max_label = ccf8bit.execute( floatImage, m_LabelImage, IntensitySimilarity<float>( maxDistance ));
  
  #ifdef CLOCK_OUTPUT
  cout << "ccf8bit: " << Clock::getInstance()->getTimestamp()-time << " ms" << endl;
  cout << "max_label: " << max_label << endl;
  time=Clock::getInstance()->getTimestamp();
  #endif
  
  return extractComponents( max_label, minSegmentSize, minOccupiedArea, expandSegmentSize );
}

std::vector< ConnectedComponent > THIS::segment(const float* floatImage, const unsigned char* mask, unsigned width, unsigned height,
                                                float maxDistance, unsigned minSegmentSize, float minOccupiedArea,
                                                unsigned expandSegmentSize, int maxr)
{
  if ( m_LabelImage )
  {
    delete[] m_LabelImage;
    m_LabelImage = 0;
  }
  m_LabelImage = new unsigned[ height * width ];
  m_Width = width;
  m_Height = height;
  
  ipcConnectedComponentFilterSim< float, unsigned, IntensitySimilarity<float>, MaskVadility<unsigned char, float> > ccf8bit;
  ccf8bit.init( width, height );
  unsigned max_label = ccf8bit.execute( floatImage, m_LabelImage, IntensitySimilarity<float>( maxDistance ),
                                        MaskVadility<unsigned char, float>( 0, mask, width ), maxr );
  
  #ifdef CLOCK_OUTPUT
  cout << "ccf8bit: " << Clock::getInstance()->getTimestamp()-time << " ms" << endl;
  cout << "max_label: " << max_label << endl;
  time=Clock::getInstance()->getTimestamp();
  #endif
  
  return extractComponents( max_label, minSegmentSize, minOccupiedArea, expandSegmentSize );
}

std::vector< ConnectedComponent > THIS::segmentByLimit( cv::Mat* pi_img, double pi_limit) {
  std::vector<ConnectedComponent> componentVec;

  unsigned width = pi_img->cols;
  unsigned height = pi_img->rows;

  // binarize the image
  unsigned char* binImage = new unsigned char[ height * width ];
  for ( unsigned j = 0; j < height; j++ ) {
    for ( unsigned i = 0; i < width; i++ ) {
      binImage [j*width+i] = ( pi_img->at<unsigned char>(i, j) >= pi_limit ? 255 : 0 );
    }
  }

  unsigned offset=0;
  unsigned char* bin2Image = new unsigned char[ height * width ];
  memcpy( bin2Image, binImage, height * width );

  // a simple erosion
  for ( unsigned j = 1; j < height-1; j++ ) {
    for ( unsigned i = 1; i < width-1; i++ ) {
    offset=j*width+i;
    bin2Image [offset] = (
        binImage [ offset ] |
        binImage [ offset-width-1 ] | binImage [ offset-width+1 ] |
        binImage [ offset+width-1 ] | binImage [ offset+width+1 ] ) == 255 ? 255 : 0;
    }
  }
  
  //apply algorithm
  componentVec = segment(bin2Image,pi_img->cols,pi_img->rows);

  delete[] binImage;
  delete[] bin2Image;

  return componentVec;
}

std::vector< ConnectedComponent > THIS::extractComponents( unsigned max_label, unsigned minSegmentSize,
                                                           float minOccupiedArea, unsigned expandSegmentSize )
{
  std::vector<ConnectedComponent> componentVec;
  unsigned* labelImage = m_LabelImage;
  unsigned width = m_Width;
  unsigned height = m_Height;
  
  //initialize variables
  unsigned* minX=new unsigned[max_label+1];
  unsigned* maxX=new unsigned[max_label+1];
  unsigned* minY=new unsigned[max_label+1];
  unsigned* maxY=new unsigned[max_label+1];
  unsigned* size=new unsigned[max_label+1];
  unsigned* sumX=new unsigned[max_label+1];
  unsigned* sumY=new unsigned[max_label+1];
  
  for (unsigned i=0;i<max_label+1;i++) {
    minX[i]=width-1;
    maxX[i]=0;
    minY[i]=height-1;
    maxY[i]=0;
    size[i]=0;
    sumX[i]=0;
    sumY[i]=0;
  }
  
  unsigned offset=0;
  unsigned currentLabel;
  unsigned y=0;
  unsigned x=0;
  unsigned realMaxLabel=0;
  unsigned currentStartX=0;
  unsigned currentSegmentSize=0;
  
  //Extract segments
  while(y<height) {
    while(x<width)
    {
      //start of line segment with label != 0 found in current line
      if (labelImage[offset]) {
        currentLabel=labelImage[offset];
        currentStartX=x;
        if (currentLabel > realMaxLabel) { realMaxLabel=currentLabel; }
        if (x<minX[currentLabel]) { minX[currentLabel]=x; }
        if (y<minY[currentLabel]) { minY[currentLabel]=y; }
        if (y>maxY[currentLabel]) { maxY[currentLabel]=y; }
        //determine end of line segment
        do {
          x++;
          offset++;
        } while ( (x<width) && (labelImage[offset]==currentLabel) );
        currentSegmentSize=x-currentStartX;
        size[currentLabel]+=currentSegmentSize;
        sumY[currentLabel]+=currentSegmentSize*y;
        sumX[currentLabel]+=(x+currentStartX)*currentSegmentSize/2;
        x--;
        offset--;
        if (x>maxX[currentLabel]) { maxX[currentLabel]=x; }
      }
      offset++;
      x++;
    }
    x=0;
    y++;
  }

  //expand & clip bounding boxes
  for (unsigned i=1;i<=realMaxLabel;i++) {
    if (minX[i]<expandSegmentSize) { minX[i]=0; }
    else { minX[i]-=expandSegmentSize; }
    if (minY[i]<expandSegmentSize) { minY[i]=0; }
    else { minY[i]-=expandSegmentSize; }
    if (maxX[i]>width-expandSegmentSize-1) { maxX[i]=width-1; }
    else { maxX[i]+=expandSegmentSize; }
    if (maxY[i]>height-expandSegmentSize-1) { maxY[i]=height-1; }
    else { maxY[i]+=expandSegmentSize; }
  }
  
  bool foundBoxes=false;
  std::ostringstream stream;
  stream << "Found bounding boxes:" << std::endl;
  //calculate centers & create ConnectedComponents
  for (unsigned i=1;i<realMaxLabel+1;i++)
  {
    float boxPixels=(maxX[i]-minX[i]-2*expandSegmentSize)*(maxY[i]-minY[i]-2*expandSegmentSize);
    float occupiedArea=float(size[i]) / boxPixels;
    
    if ( size[i]>minSegmentSize && occupiedArea>minOccupiedArea ) {
      foundBoxes=true;
      double cogX = double(sumX[i]) / double(size[i]);
      double cogY = double(sumY[i]) / double(size[i]);
      ConnectedComponent comp( i, minX[i], minY[i], maxX[i], maxY[i], cogX, cogY, size[i] );
      componentVec.push_back( comp );
      stream << i << ": " << minX[i] <<"-"<< maxX[i] <<" "<< minY[i] <<"-"<< maxY[i]
          << " center: " << int(cogX) << "," << int(cogY) // TODO << " size: " << fixed << int(size[i])
          << " width: " << (maxX[i]-minX[i]) << " height: " << (maxY[i]-minY[i])
          << " width x height: " << (maxX[i]-minX[i])*(maxY[i]-minY[i])
          << " occupied (%): " << int(occupiedArea*100.0) << std::endl;
    }
  }
  
  if (foundBoxes) {
    ROS_DEBUG_STREAM( stream.str() );
  }

  delete[] minX;
  delete[] maxX;
  delete[] minY;
  delete[] maxY;
  delete[] size;
  delete[] sumX;
  delete[] sumY;
  
  #ifdef CLOCK_OUTPUT
  cout << "extract segments: " << Clock::getInstance()->getTimestamp()-time << " ms" << endl;
  time=Clock::getInstance()->getTimestamp();
  #endif
  
  return componentVec;
}

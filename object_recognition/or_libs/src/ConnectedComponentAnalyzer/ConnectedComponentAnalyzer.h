/*******************************************************************************
 *  ConnectedComponentAnalyzer.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:  
 *  $Id: ConnectedComponentAnalyzer.h 44313 2011-04-06 22:46:28Z agas $ 
 *******************************************************************************/

#ifndef CONNECTEDCOMPONENTANALYZER_H
#define CONNECTEDCOMPONENTANALYZER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "ConnectedComponent.h"


/**
 * @class  ConnectedComponentAnalyzer
 * @author Johannes Pellenz, Patrick Lamprecht (RX), David Gossow (RX)
 *
 * @brief  Detects connected components in an image
 */
class ConnectedComponentAnalyzer {

public:

  /** Does nothing. */
  ConnectedComponentAnalyzer();

  /** Does nothing. */
  ~ConnectedComponentAnalyzer();

  /**
   * This method first detects pixels in a gray level image that
   * have a value greater equal pi_limit. Then it extracts these 
   * segments and returns a list of the segments containing 
   * bounding box, center of gravity (cog), size
   *
   * @param pi_img The pointer to a cv::Mat Gray level image.
   * @param pi_limit A pixel is considered to be part of a component if its value is equal or larger than pi_limit.
   * @return List of the segments with values above the limit
   * @author Johannes Pellenz, David Gossow
   */
  std::vector< ConnectedComponent > segmentByLimit( cv::Mat* pi_img, double pi_limit );


  /**
   * This method extracts segments of an already binarized image and returns a list of the segments containing
   * bounding box, cog, size, etc.
   *
   * @param binImage binarized image
   * @param width width of the image
   * @param height height of the image
   * @param minSegmentSize minimal segment size (pixels)
   * @param expandSegmentSize expand the segments with the given size
   * @return List of the segments with values above the limit
   * @author Patrick Lamprecht, David Gossow (RX)
   */

  std::vector< ConnectedComponent > segment(unsigned char* binImage, unsigned width, unsigned height, unsigned minSegmentSize=1, float minOccupiedArea=0.0, unsigned expandSegmentSize=0);
  
  /**
   * This method extracts segments of a range image using single linkage similarity
   * checks between components and component candidates.
   *
   * @param rangeImage range image
   * @param width width of the image
   * @param height height of the image
   * @param maxDistance the similarity threshold (pixels with intensity difference <= maxDistance belong to the same object)
   * @param minSegmentSize minimal segment size (pixels)
   * @param expandSegmentSize expand the segments with the given size
   * @return List of the segments with values above the limit
   * @author Nicolai Wojke (R14)
   */
  std::vector< ConnectedComponent > segment(const float* floatImage, unsigned width, unsigned height, float maxDistance, unsigned minSegmentSize=1, float minOccupiedArea=0.0, unsigned expandSegmentSize=0);
  std::vector< ConnectedComponent > segment(const float* floatImage, const unsigned char* mask, unsigned width, unsigned height, float maxDistance, unsigned minSegmentSize=1, float minOccupiedArea=0.0, unsigned expandSegmentSize=0, int maxr=1);

  /** Clear all mask segments except the largest one */
  static void isolateLargestSegment(unsigned char* binImage, unsigned width, unsigned height);
  
  /**
   * Get the label image and dimensions of the last segmentation run.
   */
  unsigned* getLastLabelImage() { return m_LabelImage; }
  unsigned  getImageWidth() { return m_Width; }
  unsigned  getImageHeight() { return m_Height; }
                                                       
private:
  
  /**
   * Extract connected components from a label image. Shared code between the different
   * segment() implementations
   *
   * @param labelImage label image
   * @param width image width
   * @param height image height
   * @param max_label maximum label id
   * @param minSegmentSize minimal segment size (pixels)
   * @param expandSegmentSize expand the segments with the given size
   * @return List of the segments with values above the limit
   */
  std::vector<ConnectedComponent> extractComponents(unsigned max_label, unsigned minSegmentSize,
                                                    float minOccupiedArea, unsigned expandSegmentSize);
  
  /// a similarity criterion based on intensity differences
  template < typename T >
  struct IntensitySimilarity
  {
    private:
      T m_MaxDistance;
    public:
      IntensitySimilarity( T maxDistance ) : m_MaxDistance( maxDistance ) {}
      ~IntensitySimilarity() {}
      bool operator()( const T& l, const T& r ) const
      {
        T diff;
        if ( l < r ) {
          diff = r - l ;
        } else {
          diff = l - r;
        }
        return diff <= m_MaxDistance;
      }
  };
  
  /// validity function for range image segmentation
  template < typename MaskT, typename PixelT >
  struct MaskVadility
  {
    private:
      const MaskT* mask;
      const int colstep;
      const MaskT MASKED;
      
    public:
      MaskVadility( MaskT masked, const MaskT* mask, int colstep)
      : mask(mask), colstep(colstep), MASKED(masked)
      {}
      
      bool operator()(int idx, PixelT intensity ) const
      { return mask[ idx ] != MASKED; }
      
      
  };
  
  unsigned* m_LabelImage;
  unsigned  m_Width, m_Height;
};


#endif

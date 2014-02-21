/*******************************************************************************
 *  ConnectedComponent.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  §Author: SB; DevelTest: ; Reviewer: ; Review: ; State:   
 *  
 *  Additional information:  
 *  $Id: ConnectedComponent.h 44313 2011-04-06 22:46:28Z agas $ 
 *******************************************************************************/

#ifndef CONNECTEDCOMPONENT_H
#define CONNECTEDCOMPONENT_H

/**
 * @class ConnectedComponent
 *
 * @author Johannes Pellenz
 *
 * @brief This class stores data of a connected component.
 *
 * 
 */
class ConnectedComponent{

  public:

  /**
  * Default constructor. Sets all members to 0.
  */
    ConnectedComponent()  
    {
      m_MinX = 0;
      m_MinY = 0;
      m_MaxX = 0;
      m_MaxY = 0;
      m_CogX = 0;
      m_CogY = 0;
      m_SegmentSize = 0;
      m_LabelId = 0;
    }

  
  /**
   * Constructor
   * @param pi_labelId Id of the label this component represents (0 = invalid)
   * @param pi_MinX The minimal x-coordinate of bounding-box
   * @param pi_MinY The minimal y-coordinate of bounding-box
   * @param pi_MaxX The maximal x-coordinate of bounding-box
   * @param pi_MaxY The maximal y-coordinate of bounding-box
   * @param pi_CogX The x-coordinate of the center of gravity
   * @param pi_CogY The y-coordinate of the center of gravity
   * @param pi_SegmentSize The size of the found segment in pixels
   */  
  ConnectedComponent(int pi_labelId, int pi_MinX, int pi_MinY,
      int pi_MaxX, int pi_MaxY,
      double pi_CogX, double pi_CogY,
      int pi_SegmentSize=0)
  {
    m_LabelId = pi_labelId;
    m_MinX = pi_MinX;
    m_MinY = pi_MinY;
    m_MaxX = pi_MaxX;
    m_MaxY = pi_MaxY;
    m_CogX = pi_CogX;
    m_CogY = pi_CogY;
    m_SegmentSize = pi_SegmentSize;
  }

  /**
   * Destruktor 
   */
  ~ConnectedComponent() {}

  
  /**
   * Sets coordinates of upper left corner
   * @param pi_MinX The minimal x-coordinate of bounding-box
   * @param pi_MinY The minimal y-coordinate of bounding-box
   */    
  void setMinCoordinates(int pi_MinX, int pi_MinY)
  {
    m_MinX = pi_MinX;
    m_MinY = pi_MinY;
  };
 
  /**
   * Sets coordinates of lower right corner
   * @param pi_MaxX The maximal x-coordinate of bounding-box
   * @param pi_MaxY The maximal y-coordinate of bounding-box
   */    
  void setMaxCoordinates(int pi_MaxX, int pi_MaxY)
  {
    m_MaxX = pi_MaxX;
    m_MaxY = pi_MaxY;
  };

    /**
   * Sets coordinates of the center of gravity
     */
  void setCOG(double pi_CogX, double pi_CogY)
  {
    m_CogX = pi_CogX;
    m_CogY = pi_CogY;
  };

    /**
   * @param pi_SegmentSize Size of the segment in pixels
     */    
  void setSegmentSize (int pi_SegmentSize)
  { 
    m_SegmentSize = pi_SegmentSize; 
  }

  /**
   * get Methods
   */ 
  int getMinX() const {return m_MinX;}
  int getMinY() const {return m_MinY;}
  int getMaxX() const {return m_MaxX;}
  int getMaxY() const {return m_MaxY;}
  double getCogX() const {return m_CogX;}
  double getCogY() const {return m_CogY;}
  int getSegmentSize() const {return m_SegmentSize;}
  unsigned getLabelId() const {return m_LabelId; }

private:
    /**
    * Bounding box, upper left x coordinate
    */
  int m_MinX;
    /**
   * Bounding box, upper left y coordinate
     */
  int m_MinY;
      /**
   * Bounding box, lower right x coordinate
       */
  int m_MaxX;
      /**
   * Bounding box, lower right y coordinate
       */
  int m_MaxY;
    /**
   * Center of gravity, x coordinate (sub pixel accuracy)
   */
  double m_CogX;
    /**
   * Center of gravity, y coordinate (sub pixel accuracy)
   */
  double m_CogY;
    /**
    * Size of the segment in pixels
    */
  int m_SegmentSize;
  
  unsigned m_LabelId;
};

#endif



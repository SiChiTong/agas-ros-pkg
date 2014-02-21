/*******************************************************************************
 *  KeyPointMatch.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *******************************************************************************/

#ifndef KeyPointMatch_H
#define KeyPointMatch_H

/**
 * @class  KeyPointMatch
 * @brief  Describes a match between two keypoints
 * @author David Gossow
 */
class KeyPointMatch
{
  public:

    unsigned index1;
    unsigned index2;
    double distance;

    double turnAngle;
    double scaleQuotient;

  private:

};

#endif

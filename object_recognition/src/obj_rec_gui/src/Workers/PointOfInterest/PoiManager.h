/*******************************************************************************
 *  MapManager.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  Author: DG; DevelTest: ; Reviewer: initials; Review: date State: NOK
 *
 *  Additional information:  
 *  $Id: PoiManager.h 44313 2011-04-06 22:46:28Z agas $ 
 *******************************************************************************/

#ifndef POI_MANAGER_H
#define POI_MANAGER_H

#include "PointOfInterest.h"
#include <list>


/** @class PoiManager
 * @author David Gossow
 * @brief This class manages the List of points of interest (POIs)
  *
  * This class keeps a list of all POIs within the current map. It provides the usual functions
  * to edit the list.
  */
class PoiManager {

  public:

    /** The constructor of the class. */
    PoiManager();

    /** constructor initializing the poi list */
    PoiManager( std::list< PointOfInterest > pois );

    /** Does nothing. */
    ~PoiManager() {};

    /** Adds a new POI to the list and assigns an ID
      * @param poi pointer to the PointOfInterest Object to be copied and added
      * @return ID of the new POI
      */
    int addPointOfInterest( const PointOfInterest* poi );

    /** Replaces a POI with a new one
      * @param poi pointer to the PointOfInterest Object to be copied and inserted
      *            the POI with the same ID as the new one is first deleted
      * @return true if the old POI was found and could be deleted
      *         false otherwise
      */
    bool modifyPointOfInterest( const PointOfInterest* poi );

    /** Deletes a POI with a certain ID from the list
      * @param id ID of the POI to be deleted
      * @return true if the POI was found and could be deleted
      *         false otherwise
      */
    bool deletePointOfInterest( int id );

    /** Deletes all POIs having the specified string in their name.
     * @param namePart The part of the name.
     * @return The number of PoIs deleted.
     */
    int deletePointOfInterest( std::string namePart );

    /** Returns current POI list
      * @return the POI list
      */
    std::list< PointOfInterest > getList();

  private:

    /** Adds a POI with given ID to the list
      * @param poi pointer to the PointOfInterest Object to be copied and added
      */
    bool addPointOfInterest( int id, const PointOfInterest* poi );

    /** Looks for POI with ID id in the list
      * @param id ID of the POI
      */
    bool poiExists( int id );

    /** The copy constructor of the class.
      * It's kept private, because it will never be used.
      */
    PoiManager( const PoiManager& instance );

    /** Holds the POI list */
    std::list< PointOfInterest > m_Pois;

    /** Holds a counter for the unique IDs of every PoI. */
    int m_CurrentId;

};


#undef PoiManager

#endif

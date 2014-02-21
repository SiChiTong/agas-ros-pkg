/*******************************************************************************
 *  ObjectProperties.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: ObjectProperties.h 30210 2009-03-13 15:36:58Z sisuthie $
 *******************************************************************************/


#ifndef OBJECTPROPERTIES_H
#define OBJECTPROPERTIES_H

#include <vector>
#include <string>

// #include <konihcl/OIOnih.h> // TODO get rid of this
#include "../../Workers/Puma2/GrayLevelImage8.h"

#include "../../Workers/Puma2/HistogramUV.h"
#include "../../Workers/Puma2/ColorImageUV8.h"
#include "ImageProperties.h"

/**
 * @class  ObjectProperties
 * @author Jan Bornemeier (RX), David Gossow (RX)
 * @brief  Container class for histogram and SURF descriptor of an object
 */
class ObjectProperties {

  public:

    /** @brief Default constructor. */
    ObjectProperties( std::string name="" );

    /** @brief Copy constructor */
    ObjectProperties( const ObjectProperties& other );

    /** @brief The destructor */
    ~ObjectProperties();

    /** @brief Assignment operator */
    ObjectProperties& operator= (const ObjectProperties& right);

    /** @brief Takes ownership of the given ImageProperties **/
    void addImageProperties( ImageProperties* imageProperties );

    /** @brief Set type of ObjectProperty (e.g. face or object )*/
    void setType(std::string type) { m_Type = type; }

    void setName( std::string name ) { m_Name = name; }

    // GETTER FUNCTIONS

    /** @return List of image names */
    std::vector<std::string> getImageNames();

    /** @return Object name */
    std::string getName() { return m_Name; }

    /** @return Object type */
    std::string getType() { return m_Type; }

    /** @return Mean histogram of all images */
    const HistogramUV* getMeanHistogram() const { return m_MeanHistogram; }

    const std::vector< ImageProperties* > getImageProperties( ) const { return m_ImageProperties; }

    const ImageProperties* getImageProperties( std::string name ) const;

    void deleteImageProperties( std::string name );

    void deleteImageProperties( int index );

    // TODO this is needed to load objects from hd !!!

//    /** @brief Serialize to stream */
//    void storer( ExtendedOutStream& extStrm );

//    /** @brief Deserialize from stream */
//    ObjectProperties( ExtendedInStream& extStrm );

    /** @brief Print object information */
    void printOn( std::ostream& strm );

  private:

    std::string m_Name;

    std::string m_Type;

    HistogramUV* m_MeanHistogram;

    std::vector< ImageProperties* > m_ImageProperties;
};

#endif

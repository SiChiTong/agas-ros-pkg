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

#include "ImagePropertiesCV.h"

// include headers that implement an archive in binary format
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

/**
 * @class  ObjectProperties
 * @author Jan Bornemeier (RX), David Gossow (RX)
 * @brief  Container class for histogram and SURF descriptor of an object
 */
class ObjectProperties {

  public:

    friend class boost::serialization::access;

    /** @brief Default constructor. */
    ObjectProperties( std::string name="" );

    /** @brief Copy constructor */
    ObjectProperties( const ObjectProperties& other );

    /** @brief The destructor */
    ~ObjectProperties();

    /** @brief Assignment operator */
    ObjectProperties& operator= (const ObjectProperties& right);

    /** @brief Takes ownership of the given ImageProperties **/
    void addImageProperties( ImagePropertiesCV* imageProperties );

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

    const std::vector< ImagePropertiesCV* > getImageProperties( ) const { return m_ImageProperties; }

    const ImagePropertiesCV* getImageProperties( std::string name ) const;

    void deleteImageProperties( std::string name );

    void deleteImageProperties( int index );

    // SERIALIZATION

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const
    {
        unsigned x = 12;
        ar & x;

        ar & m_Name;
        ar & m_Type;

        unsigned size = m_ImageProperties.size();
        ar & size;

        for ( unsigned i=0; i < size; i++)
        {
            ar & m_ImageProperties[i];
        }
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int version_b)
    {
        unsigned version = 0;
        ar & version;

        if ( version != 12 )
        {
          throw "Loaded object-file has wrong version number.";
        }

        ar & m_Name;
        ar & m_Type;
        unsigned size;
        ar & size;

        m_ImageProperties.reserve( size );
        for ( unsigned i=0; i < size; i++)
        {
            ImagePropertiesCV* imageProperties=new ImagePropertiesCV( );
            ar & imageProperties;
            addImageProperties( imageProperties );
        }
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

    /** @brief Print object information */
    void printOn( std::ostream& strm );

  private:

    std::string m_Name;

    std::string m_Type;

    std::vector< ImagePropertiesCV* > m_ImageProperties;
};

BOOST_CLASS_VERSION(ObjectProperties, 12)

#endif

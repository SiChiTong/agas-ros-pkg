/*******************************************************************************
 *  Config.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus
 *******************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <assert.h>
#include <cstring>

#include "../../Architecture/Thread/Mutex.h"

class TiXmlNode;
class TiXmlElement;


/**
 * @class  Config
 * @author Frank Neuhaus, David Gossow, Viktor Seib
 * @brief  Provides functionality to read from an xml configuration file
 */
class Config
{
  public:
    Config();
    Config(const Config& c2);
    Config& operator=(const Config& c2);
    ~Config();

    static Config* getInstance();
    static void setInstance ( Config* newInst )
    {
      // m_AccessMutex.lock ();
      Config* oldInst=m_inst;
      m_inst=newInst;
      delete oldInst;
      // m_AccessMutex.unlock ();
    };

    static int getInt ( const std::string& id );
    static bool getBool ( const std::string& id );
    static std::string getString( const std::string& id );
    static float getFloat( const std::string& id );

    static bool getFloatMinMax ( const std::string& id, std::pair<float,float>& minmax );
    static bool getIntMinMax ( const std::string& id, std::pair<int,int>& minmax );

    /**
      * @brief allows to load various config files and profiles besides config/default.xml and profile default
      * @param files List of files that are loaded additionally
      * @param profiles List of profiles that are loaded additionally
      */
    static void loadConfig(const std::vector<std::string> files, const std::vector<std::string> profiles, const std::string& prepend_path = ".");


    template<class T>
    void set ( const std::string& id, T f );

    bool loadFromFile ( const std::string& fileName, const std::vector<std::string>& profiles );

    void clear();

    std::vector<std::string> getAllVars() const;

    const std::vector<std::string>& getModuleList () const;

    void addModule( std::string name );

    const std::string& getFilename() const;

    std::string toText();

    std::string getActiveProfileName() const { return m_activeProfile; }

    std::string getAllProfileNames() { return m_allProfileNames; }

        /**
     * Mutex which protects getters, setters and setInstance
     */
    static Mutex m_AccessMutex;
  private:


    template<class T>
        T get ( const std::string& id ) const;

    template<class T>
        bool getMinMax ( const std::string& id, std::pair<T,T>& minmax ) const;
        
    /** helper function to parse string lists */
    std::vector<std::string> explode ( const std::string& text, const std::string& separators, bool keepSeparators=false );

    std::string m_fileName;
    std::string m_activeProfile;
    std::vector<std::string> m_moduleList;

    std::set<std::string> m_loadedProfileNames;

    std::string m_allProfileNames;

    bool parseProfile ( TiXmlNode* root );
    bool parseValues ( const std::string& level, TiXmlNode* root );
    void parseModules ( TiXmlNode* root );

    class ConfigEntry
    {
      public:
        ConfigEntry() {data=0; minval=0; maxval=0; type=TYPE_INT; }
        ~ConfigEntry() {
            if ( type==TYPE_STRING ) {
                delete[] data;
            }
        }

        ConfigEntry ( const ConfigEntry& e2 )
        {
          type=e2.type;

          minval=e2.minval;
          maxval=e2.maxval;
          if ( e2.type!=TYPE_STRING )
          {
            data=e2.data;
          }
          else
          {
            data=new char[strlen ( ( const char* ) e2.data ) +1];
            strcpy ( ( char* ) data, ( const char* ) e2.data );
          }
        }

        ConfigEntry& operator= ( const ConfigEntry& e2 )
        {
          type=e2.type;

          minval=e2.minval;
          maxval=e2.maxval;
          if ( e2.type!=TYPE_STRING )
          {
            data=e2.data;
          }
          else
          {
            data=new char[strlen ( ( const char* ) e2.data ) +1];
            strcpy ( ( char* ) data, ( char* ) e2.data );
          }
          return ( *this );
        }
        template<class T>
        void set ( T t );

        bool operator==(const ConfigEntry& e2)
        {
          //std::cout << "types: " << e2.type << " " << type << std::endl;
          if (e2.type!=type) return false;
          if (e2.type!=TYPE_STRING)
          {
            bool ret=(e2.data==data)&&(e2.minval==minval)&&(e2.maxval==maxval);;
            //std::cout << e2.data << " " << data << " returning " << ret << std::endl;
            return ret;
          }
          else
          {
            if (e2.data&&data)
            {
              return !strcmp((const char*)e2.data,(const char*)data);
            }
            else return false;
          }
        }


        /*
        template<class T>
        void set ( T t )
        {
          eType e=deduceType ( t );
          if ( e==TYPE_INT )
            setInt ( * ( int* ) &t );
          else if ( e==TYPE_FLOAT )
            setFloat ( * ( float* ) &t );
          else if ( e==TYPE_STRING )
            setString ( * ( const char** ) &t );
        }
        */

        void setInt ( int i )
        {
          type=TYPE_INT;
          data_int=i;
        }
        void setIntMin ( int m )
        {
          assert ( type==TYPE_INT );
          minval_int=m;
        }
        void setIntMax ( int m )
        {
          assert ( type==TYPE_INT );
          maxval_int=m;
        }
        void setFloat ( float f )
        {
          type=TYPE_FLOAT;
          data_float=f;
        }
        void setFloatMin ( float m )
        {
          assert ( type==TYPE_FLOAT );
          minval_float=m;
        }
        void setFloatMax ( float m )
        {
          assert ( type==TYPE_FLOAT );
          maxval_float=m;
        }
        void setString ( const char* p )
        {
          type=TYPE_STRING;
          delete[] ( char* ) data;
          data = new char[strlen ( p ) +1];
          strcpy ( ( char* ) data,p );
        }
        int getInt() const
        {
          assert ( type==TYPE_INT );
          return data_int;
        }
        float getFloat() const
        {
          assert ( type==TYPE_FLOAT );
          return data_float;
        }
        const char* getString() const
        {
          assert ( type==TYPE_STRING );
          return ( const char* ) data;
        }
        int getIntMax() const
        {
          assert ( type==TYPE_INT );
          return maxval_int;
        }
        int getIntMin() const
        {
          assert ( type==TYPE_INT );
          return minval_int;
        }
        float getFloatMax() const
        {
          assert ( type==TYPE_FLOAT );
          return maxval_float;
        }
        float getFloatMin() const
        {
          assert ( type==TYPE_FLOAT );
          return minval_float;
        }

        enum eType
        {
            TYPE_INT=1,
            TYPE_FLOAT,
            TYPE_STRING,
      };

        eType getType() const
        {
          return type;
        }

      private:

        eType deduceType ( int ) const
        {
          return TYPE_INT;
        }
        eType deduceType ( const char* ) const
        {
          return TYPE_STRING;
        }
        eType deduceType ( float ) const
        {
          return TYPE_FLOAT;
        }
        eType type;
        union
        {
          char* data;
          int data_int;
          float data_float;
        };
        union
        {
          void* minval;
          int minval_int;
          float minval_float;
        };
        union
        {
          void* maxval;
          int maxval_int;
          float maxval_float;
        };
    };

    bool parseValue ( TiXmlElement* el,class ConfigEntry& newEntry , std::string& name) const;

    void saveValue ( const std::string& name, const ConfigEntry& entry, TiXmlElement* allProfValues,TiXmlElement* values ) const;

    bool parseDocument(const std::string& fileName, const std::vector<std::string>& profiles );

    std::map<std::string, ConfigEntry> m_values;

    static Config* m_inst;

    class TiXmlDocument* m_doc;

};

template<>
inline void Config::ConfigEntry::set ( float t )
{
  setFloat ( t );
}
template<>
inline void Config::ConfigEntry::set ( int t )
{
  setInt ( t );
}
template<>
inline void Config::ConfigEntry::set ( const char* t )
{
  setString ( t );
}


/*
template<class T>
const T Config::get ( const std::string& id ) const
{
  std::map<std::string,ConfigEntry>::const_iterator it=m_values.find ( id );
  if (it==m_values.end())
  {
    std::cerr << "Config: ERROR Config value '" << id << "' not found! Exiting!" << std::endl;
  }
  assert ( it!=m_values.end() );

  ConfigEntry::eType t=it->second.getType();
  T temp;
  if ( t==ConfigEntry::TYPE_INT )
  {
    int i=it->second.getInt();
    temp=* ( T* ) &i;
  }
  else if ( t==ConfigEntry::TYPE_FLOAT )
  {
    float f=it->second.getFloat();
    temp=* ( T* ) &f;
  }
  else if ( t==ConfigEntry::TYPE_STRING )
  {
    const char* p=it->second.getString();
    temp=* ( T* ) &p;
  }
  else
    assert(0);

  return temp;
}*/
template<>
inline float Config::get ( const std::string& id ) const
  {
    m_AccessMutex.lock ();
    std::map<std::string,ConfigEntry>::const_iterator it=m_values.find ( id );
    if ( it==m_values.end() )
    {
      std::cerr << "Config: ERROR Config value '" << id << "' not found! Exiting!" << std::endl;
    }
    assert ( it!=m_values.end() );

    ConfigEntry::eType t=it->second.getType();
    if ( t!=ConfigEntry::TYPE_FLOAT )
    {
      std::cerr << "Config: ERROR Config value '" << id << "' has wrong type! Exiting!" << std::endl;
    }
    assert ( t==ConfigEntry::TYPE_FLOAT );
    float value=it->second.getFloat();
    m_AccessMutex.unlock ();
    return value;
  }

template<>
inline int Config::get ( const std::string& id ) const
  {
    m_AccessMutex.lock ();
    std::map<std::string,ConfigEntry>::const_iterator it=m_values.find ( id );
    if ( it==m_values.end() )
    {
      std::cerr << "Config: ERROR Config value '" << id << "' not found! Exiting!" << std::endl;
    }
    assert ( it!=m_values.end() );

    ConfigEntry::eType t=it->second.getType();
    if ( t!=ConfigEntry::TYPE_INT )
    {
      std::cerr << "Config: ERROR Config value '" << id << "' has wrong type! Exiting!" << std::endl;
    }
    int value=it->second.getInt();
    assert ( t==ConfigEntry::TYPE_INT );
    m_AccessMutex.unlock ();
    return value;
  }

template<>
inline const char* Config::get ( const std::string& id ) const
  {
    m_AccessMutex.lock ();
    std::map<std::string,ConfigEntry>::const_iterator it=m_values.find ( id );
    if ( it==m_values.end() )
    {
      std::cerr << "Config: ERROR Config value '" << id << "' not found! Exiting!" << std::endl;
    }
    assert ( it!=m_values.end() );

    ConfigEntry::eType t=it->second.getType();
    if ( t!=ConfigEntry::TYPE_STRING )
    {
      std::cerr << "Config: ERROR Config value '" << id << "' has wrong type! Exiting!" << std::endl;
    }
    assert ( t==ConfigEntry::TYPE_STRING );
    const char* value=it->second.getString();
    m_AccessMutex.unlock ();
    return value;
  }

template<>
inline std::string Config::get ( const std::string& id ) const
  {
    return get<const char*>(id);
  }

template<class T>
void Config::set ( const std::string& id, T f )
{
  /*if (m_values.find ( id )==m_values.end())
  {
    std::cerr << "Config: ERROR Config value '" << id << "' not found! Exiting!" << std::endl;
  }
  assert ( m_values.find ( id ) !=m_values.end() );
  */
  m_AccessMutex.lock ();
  m_values[id].set<T> ( f );
  m_AccessMutex.unlock ();
}


template<class T>
bool Config::getMinMax ( const std::string& id, std::pair<T,T>& minmax ) const
{
  std::map<std::string,ConfigEntry>::const_iterator it=m_values.find ( id );
  if ( it==m_values.end() )
  {
    std::cerr << "Config: ERROR Config value '" << id << "' not found! Exiting!" << std::endl;
  }
  assert ( it!=m_values.end() );

  ConfigEntry::eType t=it->second.getType();
  if ( t==ConfigEntry::TYPE_INT )
  {
    int mi=it->second.getIntMin();
    int ma=it->second.getIntMax();
    if ( mi==ma ) return false;
    minmax=std::make_pair ( mi,ma );
    return true;
  }
  else if ( t==ConfigEntry::TYPE_FLOAT )
  {
    float mi=it->second.getFloatMin();
    float ma=it->second.getFloatMax();
    if ( mi==ma ) return false;
    minmax=std::make_pair ( mi,ma );
    return true;
  }
  return false;
}

inline const std::string& Config::getFilename() const
{
  return m_fileName;
}

#endif

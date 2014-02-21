/*******************************************************************************
 *  Config.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus
 *******************************************************************************/

#include "Config.h"
#include <iostream>
#include "../../Architecture/TinyXML/tinyxml.h"
#include "ros/ros.h"

#include <fstream>

#define ROOTNODE_NAME "MergedConfig"

Config* Config::m_inst=0;

Mutex Config::m_AccessMutex = Mutex ();

using namespace std;

//---------------------------------------------------------------------------------
Config::Config()
{
  m_doc=0;
}
//---------------------------------------------------------------------------------
Config::~Config()
{
    delete m_doc;
}
//---------------------------------------------------------------------------------
Config::Config(const Config& c2)
  :
  m_fileName(c2.m_fileName),
  m_activeProfile(c2.m_activeProfile),
  m_moduleList(c2.m_moduleList),
  m_values(c2.m_values)
{
  if (c2.m_doc)
    m_doc=new TiXmlDocument(*c2.m_doc);
  else
    m_doc=0;
}
//---------------------------------------------------------------------------------
Config& Config::operator=(const Config& c2)
{
  m_fileName=c2.m_fileName;
  m_moduleList=c2.m_moduleList;
  m_activeProfile=c2.m_activeProfile;
  m_values=c2.m_values;
  if (c2.m_doc)
    m_doc=new TiXmlDocument(*c2.m_doc);
  else
    m_doc=0;
  return *this;
}
//---------------------------------------------------------------------------------
Config* Config::getInstance()
{
  m_AccessMutex.lock ();
  if ( !m_inst )
  {
    ROS_WARN_STREAM( "No instance was set! Trying to load 'default' profile from file 'Config.xml'.." );
    Config* newConf=new Config();
    vector<string> profiles;
    profiles.push_back("default");
    newConf->loadFromFile("Config.xml",profiles);
    setInstance( newConf );
  }
  m_AccessMutex.unlock ();
  return m_inst;
}


void Config::loadConfig(const std::vector<std::string> files, const std::vector<std::string> profiles, const std::string& prepend_path)
{
    ofstream outStream;
    std::string mergedFile = prepend_path + "/config/merged.xml";
    outStream.open(mergedFile.c_str());
    outStream << "<?xml version=\"1.0\" standalone=\"no\" ?>\n\n";
    outStream << "<MergedConfig>\n\n";

    vector< string > fileNames;
    fileNames.push_back(prepend_path + "/config/default.xml");

    // insert additional files
    std::vector<std::string>::iterator it;
    it = fileNames.end();
    fileNames.insert (it, files.begin(), files.end());

    for ( unsigned i=0; i<fileNames.size(); i++ )
    {
      ROS_INFO_STREAM( "Loading " + fileNames[i] );
      string line;
      ifstream configFile (fileNames[i].c_str());
      if (configFile.is_open()) {
        while ( configFile.good() )
        {
          getline (configFile,line);
          outStream << line << endl;
        }
        configFile.close();
      }
    }

    outStream << "\n\n</MergedConfig>";
    outStream.close();

    vector<std::string> profilesToLoad;
    profilesToLoad.push_back("default");

    // insert additional profiles
    it = profilesToLoad.end();
    profilesToLoad.insert (it, profiles.begin(), profiles.end());

    Config* newConf=new Config();
    if (!newConf->loadFromFile( prepend_path + "/config/merged.xml", profilesToLoad ))
    {
        ROS_ERROR_STREAM("Could not load config file!");
        exit ( -126 );
    }
    Config::setInstance(newConf);
}


int Config::getInt ( const std::string& id )
{
  return getInstance()->get<int>( id );
}

bool Config::getBool ( const std::string& id )
{
  return getInstance()->get<int>( id );
}

std::string Config::getString ( const std::string& id )
{
  return getInstance()->get<const char*>( id );
}

float Config::getFloat ( const std::string& id )
{
  return getInstance()->get<float>( id );
}

bool Config::getFloatMinMax ( const std::string& id, std::pair<float,float>& minmax )
{
  return getInstance()->getMinMax<float>( id, minmax );
}

bool Config::getIntMinMax ( const std::string& id, std::pair<int,int>& minmax )
{
  return getInstance()->getMinMax<int>( id, minmax );
}

//---------------------------------------------------------------------------------
void Config::clear()
{
  m_AccessMutex.lock ();
  m_values.clear();
  m_AccessMutex.unlock ();
}
//---------------------------------------------------------------------------------
bool Config::parseValue ( TiXmlElement* el,Config::ConfigEntry& newEntry, std::string& name ) const
{
  if ( !el )
    return false;

  bool error=false;

  const char* pName=el->Attribute ( "name" );
  if ( !pName )
    return false;

  /*std::string */name=pName;

  if ( name.size() <1 )
    return false;

  //ConfigEntry newEntry;
  if ( name.c_str() [0]=='i' || name.c_str() [0]=='b' )
  {
    int tmp=0;
    error&= ( el->QueryIntAttribute ( "value",&tmp ) !=TIXML_SUCCESS );
    newEntry.setInt ( tmp );

    tmp=0;
    error&= ( el->QueryIntAttribute ( "min", &tmp ) !=TIXML_SUCCESS );
    newEntry.setIntMin ( tmp );

    tmp=0;
    error&= ( el->QueryIntAttribute ( "max", &tmp ) !=TIXML_SUCCESS );
    newEntry.setIntMax ( tmp );
  }
  else if ( name.c_str() [0]=='f' )
  {
    float tmp=0;
    error&= ( el->QueryFloatAttribute ( "value", &tmp ) !=TIXML_SUCCESS );
    newEntry.setFloat ( tmp );

    tmp=0;
    error&= ( el->QueryFloatAttribute ( "min", &tmp ) !=TIXML_SUCCESS );
    newEntry.setFloatMin ( tmp );

    tmp=0;
    error&= ( el->QueryFloatAttribute ( "max", &tmp ) !=TIXML_SUCCESS );
    newEntry.setFloatMax ( tmp );
  }
  else if ( name.c_str() [0]=='s' )
  {
    const char* val=el->Attribute ( "value" );
    if ( val )
    {
      newEntry.setString ( val );
    }
    else
      error=true;
  }
  else
    error=true;

  if ( error )
    return false;

  //m_values[level+"."+name]=newEntry;

  return true;
}
//---------------------------------------------------------------------------------
bool Config::parseValues ( const std::string& level, TiXmlNode* root )
{
  if ( !root )
    return false;

  for ( TiXmlNode* value=root->FirstChildElement();value!=NULL;value=value->NextSibling() )
  {
    TiXmlElement* el=value->ToElement();
    if (!el) continue;
    if ( !strcasecmp ( el->Value(),"Modules" ) ) continue;
    if ( !strcasecmp ( el->Value(),"value" ) )
    {
      ConfigEntry dest;
      std::string name;
      if (parseValue ( el,dest ,name))
      {
        m_values[level+"."+name]=dest;
      }
    }
    else
    {
      if ( !level.empty() )
        parseValues ( level+"."+el->Value(),value );
      else
        parseValues ( el->Value(),value );
    }

  }

  return true;
}
//---------------------------------------------------------------------------------
void Config::addModule( std::string name )
{
  bool found=false;
  for (unsigned i=0; i<m_moduleList.size(); i++ )
  {
    if (m_moduleList[i] == name)
    {
      found=true;
      break;
    }
  }
  if ( !found )
  {
    ROS_DEBUG_STREAM( "Adding module "+name );
    m_moduleList.push_back( name );
  } else {
    ROS_WARN_STREAM( "Duplicate module entry '"+name+"' was ignored." );
  }
}
//---------------------------------------------------------------------------------
void Config::parseModules(TiXmlNode* root)
{
  if (!root)
    return;

  for ( TiXmlNode* value=root->FirstChildElement();value!=NULL;value=value->NextSibling() )
  {
    if (!value->ToElement()) continue;
    if (!strcasecmp(value->ToElement()->Value(),"module"))
    {
      if (value->ToElement()->Attribute("name"))
      {
        std::string name=value->ToElement()->Attribute("name");
        addModule( name );
      }
    }
  }

}

bool Config::parseProfile ( TiXmlNode* root )
{
  //Parse parent profile
  if ( root->ToElement()->Attribute("parents") )
  {
    string parents = root->ToElement()->Attribute("parents");
//     ROS_INFO( parents );
    std::vector<std::string> parentList = explode( parents, "," );
    
    for ( unsigned i=0; i < parentList.size(); i++ )
    {
      string parentProfileName = parentList[i];
      TiXmlNode* docRoot=m_doc->FirstChild ( ROOTNODE_NAME );
      TiXmlNode* parentProfileNode=docRoot->FirstChild ( parentProfileName );
      
      if ( m_loadedProfileNames.find( parentProfileName) != m_loadedProfileNames.end() )
      {
        ROS_ERROR_STREAM( "Cyclic profile dependecy detected! Main profile: "+m_activeProfile );
        ROS_INFO_STREAM( "Profile " << parentProfileName << " already loaded" );
        return true;
      }
      
      if (!parentProfileNode) {
        ROS_ERROR_STREAM( "Parent profile node '" + parentProfileName + "' not found");
        return false;
      }
      
      m_loadedProfileNames.insert( parentProfileName );
      parseProfile( parentProfileNode );
    }
    
  }

  ROS_INFO_STREAM( "Parsing profile "+string(root->ToElement()->Value()) );

  TiXmlNode* modules=root->FirstChild ( "Modules" );
  if (modules) {
    parseModules(modules);
  }

  if ( !parseValues( "", root) ) {
     ROS_ERROR_STREAM( "Could not parse values from " + root->ValueStr()); 
    return false;
  }
  return true;
}

//---------------------------------------------------------------------------------
bool Config::parseDocument(const std::string& fileName, const std::vector<std::string>& profiles )
{
  m_fileName=fileName;

  TiXmlNode* root=m_doc->FirstChild ( ROOTNODE_NAME );
  if ( !root ) {
    ROS_ERROR_STREAM( "Failed to load " + fileName + ": Root node not found: " << ROOTNODE_NAME);
    return false;
  }

  //Read list of all profiles
  TiXmlNode* child = 0;
  ostringstream stream;
  stream.setf ( ios::left, ios::adjustfield );
  stream.width(35);
  stream << "Profile";
  stream << "Parents";
  stream << endl << "---------------------------------------------------------";
  while( (child = root->IterateChildren( child )) )
  {
    if ( child->Type() == TiXmlNode::ELEMENT )
    {
      string parentName=" ";
      if ( child->ToElement()->Attribute("parents") )
      {
        parentName=child->ToElement()->Attribute("parents");
      }
      stream << endl;
      stream.width(35);
      stream << child->ValueStr();
      stream << parentName;
    }
  }
  m_allProfileNames=stream.str();

  stream.str("");
  //load all profiles
  for ( unsigned p=0; p<profiles.size(); p++ )
  {
    string profileName = profiles[p];
    
    if ( p != 0 )
    {
      stream << "_";
    }
    stream << profileName;
    
    TiXmlNode* profileNode=root->FirstChild ( profileName );
    if (!profileNode) {
      ROS_ERROR_STREAM( "Failed to load " + fileName + ": Profile node '" + profileName + "' not found\n\n"+m_allProfileNames);
      return false;
    }

    m_loadedProfileNames.insert( profileName );
    if (!parseProfile( profileNode )) {
      ROS_ERROR_STREAM( "Failed to load " + fileName + ": Could not parse profile '"+profileName+"'\n\n"+m_allProfileNames);
      return false;
    }
  }
  
  m_activeProfile = stream.str();

  return true;
}

//---------------------------------------------------------------------------------
bool Config::loadFromFile ( const std::string& fileName, const std::vector<std::string>& profiles )
{
  delete m_doc;
  m_doc=new TiXmlDocument ( fileName.c_str() );
  bool loadOkay = m_doc->LoadFile();
  if ( !loadOkay )
  {
    ROS_ERROR_STREAM( "Failed to load " + fileName + ": "+ m_doc->ErrorDesc());
    return false;
  }
  return parseDocument(fileName,profiles);

}
//---------------------------------------------------------------------------------
void Config::saveValue ( const std::string& name, const ConfigEntry& entry, TiXmlElement* allProfValues,TiXmlElement* values ) const
{
  assert(values);
  std::string valuename;

  std::string::size_type start=0;
  std::string::size_type pos;
  TiXmlElement* curAllProf=allProfValues;
  bool doInsert=false;
  while ( 1 )
  {
    pos = name.find ( '.' ,start);
    if ( pos == std::string::npos )
    {
      valuename=name.substr ( start );
      curAllProf=curAllProf->FirstChildElement();
      while (curAllProf)
      {
        const char* nameattr=curAllProf->Attribute("name");
        if (!nameattr) continue;
        if (!strcmp(curAllProf->Attribute("name"),valuename.c_str()))
          break;
        curAllProf=curAllProf->NextSiblingElement();
      }
      if (!curAllProf)
      {
        doInsert=true;
      }
      break;
    }
    else
    {
      TiXmlElement* childAll=curAllProf->FirstChildElement ( name.substr ( start,pos-start ).c_str() );
      if ( !childAll )
      {
        doInsert=true;
        break;
      }
      curAllProf=childAll;
      start=pos+1;
    }
  }
  TiXmlElement * cxn = new TiXmlElement ( "value" );


  cxn->SetAttribute ( "name", valuename.c_str() );
  if ( entry.getType() ==ConfigEntry::TYPE_INT )
  {
    char tmp[256];
    sprintf ( tmp,"%d",entry.getInt() );
    cxn->SetAttribute ( "value",tmp );

    if ( entry.getIntMax() !=entry.getIntMin() )
    {
      sprintf ( tmp,"%d",entry.getIntMin() );
      cxn->SetAttribute ( "min",tmp );
      sprintf ( tmp,"%d",entry.getIntMax() );
      cxn->SetAttribute ( "max",tmp );
    }
  }
  else if ( entry.getType() ==ConfigEntry::TYPE_FLOAT )
  {
    char tmp[256];
    sprintf ( tmp,"%1.2f",entry.getFloat() );
    cxn->SetAttribute ( "value",tmp );

    if ( entry.getFloatMax() !=entry.getFloatMin() )
    {
      sprintf ( tmp,"%1.2f",entry.getFloatMin() );
      cxn->SetAttribute ( "min",tmp );
      sprintf ( tmp,"%1.2f",entry.getFloatMax() );
      cxn->SetAttribute ( "max",tmp );
    }
  }
  else if ( entry.getType() ==ConfigEntry::TYPE_STRING )
  {
    cxn->SetAttribute ( "value",entry.getString() );
  }
  else
    assert ( 0 );


  if (!doInsert)
  {
    TiXmlElement* val=curAllProf;
    if (val)
    {
      ConfigEntry ent;
      std::string dummy;
      if (parseValue(val,ent,dummy))
      {
        if (ent==entry)
        {
          delete cxn;
          return;
        }
      }

    }
  }

  start=0;
  TiXmlElement* cur=values;
  while ( 1 )
  {
    pos = name.find ( '.' ,start);
    if ( pos == std::string::npos )
    {
      valuename=name.substr ( start );
      break;
    }
    else
    {
      TiXmlElement* child=cur->FirstChildElement ( name.substr ( start,pos-start ).c_str() );
      if ( !child )
      {
        child=new TiXmlElement ( name.substr ( start,pos-start ).c_str() );
        cur->LinkEndChild ( child );
      }
      cur=child;

      start=pos+1;

    }
  }


  cur->LinkEndChild ( cxn );

}
//---------------------------------------------------------------------------------
std::vector<std::string> Config::getAllVars() const
{
  std::vector<std::string> ret;
  for ( std::map<std::string,ConfigEntry>::const_iterator it=m_values.begin();it!=m_values.end();++it )
  {
    ret.push_back(it->first);
  }
  return ret;
}
//---------------------------------------------------------------------------------
const std::vector<std::string>& Config::getModuleList() const
{
  return m_moduleList;
}
//---------------------------------------------------------------------------------


std::vector<std::string> Config::explode (const std::string& text, const std::string& separators, bool keepSeparators)
{
  size_t n     = text.length ();
  size_t start = text.find_first_not_of (separators);
  std::vector<std::string> words;
  
  while (start < n)
  {
    size_t stop = text.find_first_of (separators, start);
    if (stop > n) stop = n;
    if ( keepSeparators )
    {
      words.push_back (text.substr (start, stop-start+1));
    }
    else
    {
      words.push_back (text.substr (start, stop-start));
    }
    start = text.find_first_not_of (separators, stop+1);
  };
  return words;
}
//---------------------------------------------------------------------------------

/*******************************************************************************
 *  ORLoaderModule.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#include "ORLoaderModule.h"
#include "ORControlModule.h"
#include "ORMatchingModule.h"

#include "Architecture/Config/Config.h"

#include "ObjectRecognition/ObjectProperties.h"
#include "Workers/String/String.h"

#include <sstream>
#include <fstream>

#include <ros/package.h>


#define THIS ORLoaderModule

THIS::THIS(ros::NodeHandle *nh, ORMatchingModule* objRecMatchingModule)
{
    m_ORMatchingModule = objRecMatchingModule;

    // subscribe to messages
    m_ORCommandSubscriber = nh->subscribe<or_msgs::OrCommand>("/or/commands", 10, &ORLoaderModule::callbackOrCommand, this);

    loadDefaultObjects();
}

THIS::~THIS()
{}


void THIS::callbackOrCommand( const or_msgs::OrCommand::ConstPtr& or_command_msg )
{
    switch( or_command_msg->command )
    {
      case ORControlModule::LoadObject:
        loadObjectProperties( or_command_msg->string_value );
        break;

      default:
        break;
    }
}

void THIS::loadDefaultObjects( )
{
  std::string objectListStr = Config::getString( "ObjectRecognition.sLoadObjects" );
  ROS_INFO_STREAM( "Loading ObjectRecognition.sLoadObjects: " <<objectListStr <<"\n");

  std::vector<std::string> objectList = String::explode( objectListStr, ",;" );
  for ( unsigned i=0; i<objectList.size(); i++ )
  {
    loadObjectProperties( objectList[i] );
  }
}


void THIS::loadObjectProperties( std::string filename )
{
//    std::string dir = Config::getString( "ObjectRecognition.sDataPath" );
//    filename = dir + filename + ".objprop";

   std::string dir = Config::getString( "ObjectRecognition.sDataPath" );
   std::string path = ros::package::getPath("or_nodes");
   filename = path + dir + filename + ".objprop";

   ROS_INFO_STREAM("dir: " << dir);
   ROS_INFO_STREAM("path: " << path);
   ROS_INFO_STREAM("filename: " << filename);

  // Return if file does not exist
  if(!fileExists(filename))
  {
      ROS_ERROR_STREAM("Loading object properties. File not found: " + filename);
      return;
  }

  ObjectProperties* objectProperties;

  try
  {
    ROS_INFO_STREAM( "Loading " + filename );
    std::ifstream ifs( filename.c_str() );
    boost::archive::text_iarchive ia(ifs);
    objectProperties = new ObjectProperties();
    ia >> objectProperties;
    ifs.close();
  }
  catch ( const char* c )
  {
    ROS_ERROR_STREAM( "Cannot load object: " << c );
    return;
  }

  m_ORMatchingModule->addObjectProperties(objectProperties);
}



bool THIS::fileExists(const std::string& file)
{
    struct stat buf;
    if (stat(file.c_str(), &buf) != -1)
    {
        return true;
    }
    return false;
}



#undef THIS

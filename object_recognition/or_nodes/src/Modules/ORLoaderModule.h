/*******************************************************************************
 *  ORLoaderModule.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *******************************************************************************/

#ifndef ORLoaderModule_H
#define ORLoaderModule_H

#include <sys/stat.h>

#include <ros/ros.h>
#include <or_msgs/OrCommand.h>

class ORMatchingModule;

/**
 * @class  ORLoaderModule
 * @brief  Loads the object descriptors from disk
 * @author David Gossow (RX), Viktor Seib (R20)
 */
class ORLoaderModule
{
  public:

    /** @brief The constructor. */
    ORLoaderModule(ros::NodeHandle *nh, ORMatchingModule* objRecMatchingModule);

    /** @brief The destructor. */
    virtual ~ORLoaderModule();

  private:

    void callbackOrCommand( const or_msgs::OrCommand::ConstPtr& msg );

    ros::Subscriber m_ORCommandSubscriber;

    ORMatchingModule* m_ORMatchingModule;

    void loadDefaultObjects( );
    void loadObjectProperties( std::string filename );
    bool fileExists(const std::string& file);
};

#endif


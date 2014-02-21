#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>

#include "Modules/ORControlModule.h"
#include "Modules/ORLoaderModule.h"
#include "Modules/ORMatchingModule.h"

#include "Architecture/Config/Config.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obj_rec");
    ros::NodeHandle nh;

    // load config
    std::string path = ros::package::getPath("or_nodes");
    std::vector<std::string> fileNames;
    std::vector<std::string> profilesToLoad;
    // read params from parameter server
    if(ros::param::has("/OrNodes/sConfigFile") && ros::param::has("/OrNodes/sProfile"))
    {
        std::string configFile;
        ros::param::get("/OrNodes/sConfigFile", configFile);
        fileNames.push_back(path + configFile);

        std::string profile;
        ros::param::get("/OrNodes/sProfile", profile);
        profilesToLoad.push_back(profile);
    }
    else
    {
        std::string defaultConf = path + "/config/custom.xml";
        std::string defaultProfile = "drinks";
        ROS_WARN_STREAM("No Parameter \"/OrNodes/sConfigFile\" or \"/OrNodes/sProfile\" found.\nLoading default config file: \""+defaultConf+"\" and default profile: \""+defaultProfile+"\"");
        fileNames.push_back(defaultConf);
        profilesToLoad.push_back(defaultProfile);
    }
    Config::loadConfig(fileNames, profilesToLoad, path);


    // read input image topic
    std::string inputTopic;
    if(ros::param::has("/OrNodes/sInputImageTopic"))
    {
        ros::param::get("/OrNodes/sInputImageTopic", inputTopic);
        ROS_INFO_STREAM("Using topic " + inputTopic + " for image input.");
    }
    else
    {
        inputTopic = "/camera/rgb/image_color";
        ROS_WARN_STREAM("No parameter \"/OrNodes/sInputImageTopic\" found. Using default topic " + inputTopic + " for image input.");
    }


    ORMatchingModule* objRecMatchingModule = new ORMatchingModule(&nh, inputTopic);
    ORControlModule* objRecControlModule = new ORControlModule(&nh, objRecMatchingModule);
    ORLoaderModule* objRecLoaderModule = new ORLoaderModule(&nh, objRecMatchingModule);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete objRecLoaderModule;
    delete objRecControlModule;
    delete objRecMatchingModule;

    return 0;
}

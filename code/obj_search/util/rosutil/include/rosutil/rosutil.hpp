/**
 * @file   rosutil.hpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Mon Mar  9 13:37:24 2015
 * 
 * @brief Functions for better ROS interaction.
 * 
 * 
 */

#include <ros/ros.h>

namespace ROSUtil{
    template<typename T>
    void getParamGeneric(ros::NodeHandle handle, std::string paramName, T &paramVar);
    void getParam(ros::NodeHandle handle, std::string paramName, int &paramVar);
    void getParam(ros::NodeHandle handle, std::string paramName, double &paramVar);
    void getParam(ros::NodeHandle handle, std::string paramName, std::string &paramVar);
    void getParam(ros::NodeHandle handle, std::string paramName, float &paramVar);
    void getParam(ros::NodeHandle handle, std::string paramName, bool &paramVar);
    template<typename T>
    void getParamGenericVec(ros::NodeHandle handle, std::string paramName, std::vector<T> &paramVar);
    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<std::string> &paramVar);
    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<bool> &paramVar);
    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<int> &paramVar);
    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<float> &paramVar);
    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<double> &paramVar);
};

/**
 * @file   rosutil.cpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Mon Mar  9 13:37:03 2015
 * 
 * @brief Implementation of functions for better ros interaction.
 * 
 * 
 */

#include "rosutil.hpp"

/**
 * @namespace ROSUtil Utilities for easier interaction with ROS.
 */
namespace ROSUtil {
    
    /**
     * Get a parameter from the parameter server with name paramName and type T, and
     * assign its value to paramVar. If there is no parameter on the server with
     * that name, print an error and stop execution.
     *
     * This function is called by the other non-vector \p getParam() functions.
     * 
     * @param handle The NodeHandle for the ROS node
     * @param paramName The name of the parameter value to retrieve from the server
     * @param paramVar The variable into which to load the value of the
     * parameter. This should correspond to the type of the parameter on the
     * server. You can always extract parameters as strings.
     */
    template<typename T>
    void getParamGeneric(ros::NodeHandle handle, std::string paramName, T &paramVar) {
	if (!handle.getParam(paramName, paramVar)){
	    ROS_ERROR_STREAM("Parameter " << paramName << " has not been defined!");
	    std::exit(1);
	}
	ROS_INFO_STREAM("Successfully loaded param " << paramName << " with value " << paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, int &paramVar) {
	getParamGeneric(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, bool &paramVar) {
	getParamGeneric(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, double &paramVar) {
	getParamGeneric(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::string &paramVar) {
	getParamGeneric(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, float &paramVar) {
	double tmp;
	getParamGeneric(handle, paramName, tmp);
	paramVar = tmp;
    }


    /** 
     * Get a vector parameter from the parameter server with name \p paramName
     * and vector type \p T, and assign its value to the vector \p paramVar. If
     * there is no parameter on the server with that name, print an error and
     * stop execution.
     *
     * This function is called by all the other vector \p getParam() functions.
     *
     * @param handle The NodeHandle for the ROS node
     * @param paramName The name of the parameter value to retrieve from the server
     * @param paramVar The variable into which to load the value of the
     * parameter. This should correspond to the type of the parameter on the
     * server. You can always extract parameters as strings.
     */
    template<typename T>
    void getParamGenericVec(ros::NodeHandle handle, std::string paramName, std::vector<T> &paramVar) {
	if (!handle.getParam(paramName, paramVar)){
	    ROS_ERROR_STREAM("Parameter " << paramName << " has not been defined!");
	    std::exit(1);
	}
	ROS_INFO_STREAM("Successfully loaded vector param " << paramName);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<std::string> &paramVar) {
	getParamGenericVec(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<bool> &paramVar) {
	getParamGenericVec(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<int> &paramVar) {
	getParamGenericVec(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<float> &paramVar) {
	getParamGenericVec(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<double> &paramVar) {
	getParamGenericVec(handle, paramName, paramVar);
    }

}

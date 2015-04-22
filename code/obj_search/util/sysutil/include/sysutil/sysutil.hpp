/**
 * @file   sysutil.hpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Tue Mar 10 14:52:01 2015
 * 
 * @brief  
 * 
 * 
 */

#ifndef SYSUTIL_H
#define SYSUTIL_H

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <iostream>
#include <regex>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace SysUtil {

    /** 
     * Store information about files and directories in a given directory.
     */
    struct DirContents {
	/// Path to the directory that was listed
	std::string path;
	/// Directories in the directory
	std::vector<std::string> dirs;
	/// Files in the directory
	std::vector<std::string> files;
	/// @brief If directories are listed recursively, can use this to check
	/// if this was the top level directory
	bool isTop;
    };
    
    bool isType(const std::string& path, mode_t mode);
    bool isDir(const std::string& path);
    bool isFile(const std::string& path);
    DirContents listDir(std::string path);
    std::vector<std::string> listFilesWithString(std::string path, std::regex r);
    std::vector<std::string> listFilesWithString(std::string path, std::string s);
    std::string cleanDirPath(std::string path);
    std::string fullDirPath(std::string path);
    std::string removePathBase(std::string path);
    std::string trimPath(std::string path, int nToTrim, bool fromFront=false);
    std::string removeExtension(std::string filename, bool trim=true);
    bool makeDirs(std::string path);
    std::string combinePaths(std::string a, std::string b);
} // namespace SysUtil

#endif // SYSUTIL_H

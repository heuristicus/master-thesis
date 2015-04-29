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
#include <ctime>
#include <dirent.h>
#include <iostream>
#include <regex>
#include <string>
#include <sys/stat.h>
#include <vector>
#include <queue>

namespace objsearch {
    namespace sysutil {

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
	    bool recursive;
	};
    
	bool isType(const std::string& path, mode_t mode);
	bool isDir(const std::string& path);
	bool isFile(const std::string& path);
	DirContents listDir(std::string path, bool recursive=false);
	std::vector<std::string> listFilesWithString(std::string path, std::regex r,
						     bool recursive=false);
	std::vector<std::string> listFilesWithString(std::string path, std::string s,
						     bool recursive=false);
	std::string cleanDirPath(std::string path);
	std::string fullDirPath(std::string path);
	std::string removePathBase(std::string path);
	std::string trimPath(std::string path, int nToTrim, bool fromFront=false);
	std::string removeExtension(std::string filename, bool trim=true);
	bool makeDirs(std::string path);
	std::string combinePaths(std::string a, std::string b);
	std::string getDateTimeString();
    } // namespace sysutil
} // namespace objsearch

#endif // SYSUTIL_H


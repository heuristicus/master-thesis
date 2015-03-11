/**
 * @file   sysutil.hpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Tue Mar 10 14:52:01 2015
 * 
 * @brief  
 * 
 * 
 */
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include <cerrno>
#include <cstdio>
#include <vector>



namespace SysUtil {
    bool isType(const std::string& path, mode_t mode);
    bool isDir(const std::string& path);
    bool isFile(const std::string& path);
    std::string cleanDirPath(std::string path);
    std::string fullDirPath(std::string path);
    std::string removePathBase(std::string path);
    std::string trimPath(std::string path, int nToTrim, bool fromFront=false);
    bool makeDirs(std::string path);
    std::string combinePaths(std::string a, std::string b);
} // namespace SysUtil

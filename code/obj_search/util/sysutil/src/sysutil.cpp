/**
 * @file   sysutil.cpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Tue Mar 10 11:10:10 2015
 * 
 * @brief  
 * 
 * 
 */
#include "sysutil.hpp"
#include <iostream>

/**
 * @namespace SysUtil Namespace for system utilities like directory interaction
 */
namespace SysUtil {
    
    /** 
     * Check if the path given is of a certain type.
     *
     * See [the GNU documentation](
     * http://www.gnu.org/software/libc/manual/html_node/Testing-File-Type.html#Testing-File-Type)
     * for possible types.
     * 
     * @param path The path to check.
     * @param mode Check if the path is of this mode type.
     * 
     * @return True if \p path is of the given \p mode.
     */
    bool isType(const std::string& path, mode_t mode) {
	struct stat s;
	if (stat(path.c_str(), &s) == 0) {
	    if (s.st_mode & mode) {
		return true;
	    } else {
		return false;
	    }
	} else {
	    return false;
	}
    }

    /** 
     * Check if the given path is a directory.
     * 
     * @param path Path to check.
     * 
     * @return True if \p path is a directory.
     */
    bool isDir(const std::string& path) {
	return isType(path, S_IFDIR);
    }

    /** 
     * Check if the given path is a file.
     * 
     * @param path Path to check.
     * 
     * @return True if \p path is a file.
     */
    bool isFile(const std::string& path) {
	return isType(path, S_IFREG);
    }


    /**
     * Removes the trailing slash from a directory path if there is one.
     * 
     * For example, `/home/user/data/stuff/` would be converted to `/home/user/data/stuff`
     * 
     * @param path Path string to clean up.
     * 
     * @return \p path with the last character  a `/`, 
     */
    std::string cleanDirPath(std::string path) {
	std::string dir = path;
	if (path[path.size() - 1] == '/'){
	    dir = path.substr(0, dir.size() - 1);
	}
	return dir;
    }

    /**
     * Ensures that the given path ends with a `/` if it does not already. If
     * the path provided is not a directory, the original string is returned.
     * Assumes that the path given is actually a directory and not a file. If it
     * is a file you will end up with a trailing slash on that.
     *
     * For example, `/home/user/data/stuff` would be converted to `/home/user/data/stuff/`
     * 
     * @param path The path string to extend.
     * 
     * @return \p path with `/` as the last character.
     */
    std::string fullDirPath(std::string path) {
	std::string dir = path;
	if (path[path.size() - 1] != '/'){
	    dir = dir + "/";
	}
	return dir;
    }

    /** 
     * Trim the given path back by \p nToTrim elements. A path element is either
     * a file or a directory.
     *
     * The \p path `/home/user/data/file.txt` has four elements. Three are
     * directories and one is a filename. The elements are trimmed from the end
     * of the path by default.
     * 
     * @param path The path to trim
     * @param nToTrim The number of elements to trim from the path.
     * @param fromFront When true, trim the elements from the front of the path
     * instead of the back.
     * 
     * @return A path with the given number of elements trimmed. If \p nToTrim
     * exceeds the total number of elements, then you will be left with either a
     * filename if \p fromFront is true, or an empty string otherwise. Trimming
     * `/home/user/data/file.txt` by 2 elements from the back would give you
     * `/home/user`. Notice that there is no trailing slash. Going from the
     * front the return would be `data/file.txt`.
     */
    std::string trimPath(std::string path, int nToTrim, bool fromFront){
	if (nToTrim <= 0) {
	    return path;
	}

	int nextDir;
	if (fromFront) {
	    // going from the front, so need to ignore ~/ or / at the start of paths.

	    if (path[0] == '/') {
		// if starting from the root, skip the first character of the
		// path
		path = std::string(path, 1);
	    } 
	    
	    // could consider these cases, but they are actually directories,
	    // and shouldn't be ignored
	    // else if (path[0] == '~') {
	    // 	// if starting from the home directory, skip the first two
	    // 	// characters of the path (~/)
	    // 	path = std::string(path, 2);
	    // } else if (std::string("../").compare(path, 0)) {
	    // } 

	    while ((nextDir = path.find_first_of("/")) != std::string::npos
		   && nToTrim-- > 0) {
		path = std::string(path, nextDir + 1);
	    }
	} else {
	    while ((nextDir = path.find_last_of("/")) != std::string::npos
		   && nToTrim-- > 0) {
		path = std::string(path, 0, nextDir);
	    }	    
	}

	return path;
    }
    
    /**
     * Removes the base of a path, leaving either the directory or the file
     * name.
     *
     * For example, if \p path is `/home/user/data/file.txt`, return is
     * `file.txt`. If \p path is `/home/user/data/` then the return is `data`.
     * 
     * @param path Path to trim
     * 
     * @return A filename or directory name corresponding to the last name in the path.
     */
    std::string removePathBase(std::string path) {
	std::string cleanPath = cleanDirPath(path);
	int lastDir = cleanPath.find_last_of("/");
	return cleanPath.substr(lastDir + 1);
    }

} // namespace SysUtil

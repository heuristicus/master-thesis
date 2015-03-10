#include "sysutil/sysutil.hpp"
#include <gtest/gtest.h>

namespace testing {
    namespace sysutil {

	TEST(sysutilTest, cleanDirPath) {
	    ASSERT_STREQ(std::string("/test/the/function").c_str(),
			 SysUtil::cleanDirPath("/test/the/function/").c_str());
	    ASSERT_STREQ(std::string("/test/the/function").c_str(),
			 SysUtil::cleanDirPath("/test/the/function").c_str());
	}

	TEST(sysutilTest, fullDirPath) {
	    ASSERT_STREQ(std::string("/test/the/function/").c_str(),
			 SysUtil::fullDirPath("/test/the/function").c_str());
	    ASSERT_STREQ(std::string("/test/the/function/").c_str(),
			 SysUtil::fullDirPath("/test/the/function/").c_str());
	}

	TEST(sysutilTest, isFile) {
	    ASSERT_TRUE(SysUtil::isFile("/usr/bin/g++"));
	    ASSERT_FALSE(SysUtil::isFile("/usr/bin"));
	}

	TEST(sysutilTest, isDir) {
	    ASSERT_TRUE(SysUtil::isDir("/usr/bin"));
	    ASSERT_FALSE(SysUtil::isDir("/usr/bin/g++"));
	}
	
	TEST(sysutilTest, trimPath_homedir) {
	    ASSERT_STREQ(std::string("~/hello/this").c_str(),
			 SysUtil::trimPath("~/hello/this/is/my", 2).c_str());
	    ASSERT_STREQ(std::string("this/is/my").c_str(),
			 SysUtil::trimPath("~/hello/this/is/my", 2, true).c_str());

	    ASSERT_STREQ(std::string("~/hello/this/is").c_str(),
			 SysUtil::trimPath("~/hello/this/is/my/file.txt", 2).c_str());
	    ASSERT_STREQ(std::string("this/is/my/file.txt").c_str(),
			 SysUtil::trimPath("~/hello/this/is/my/file.txt", 2, true).c_str());
	}

	TEST(sysutilTest, trimPath_rootdir) {
	    ASSERT_STREQ(std::string("/hello/this").c_str(),
			 SysUtil::trimPath("/hello/this/is/my", 2).c_str());
	    ASSERT_STREQ(std::string("is/my").c_str(),
			 SysUtil::trimPath("/hello/this/is/my", 2, true).c_str());

	    ASSERT_STREQ(std::string("/hello/this/is").c_str(),
			 SysUtil::trimPath("/hello/this/is/my/file.txt", 2).c_str());
	    ASSERT_STREQ(std::string("is/my/file.txt").c_str(),
			 SysUtil::trimPath("/hello/this/is/my/file.txt", 2, true).c_str());
	}

	TEST(sysutilTest, trimPath_noslash) {
	    ASSERT_STREQ(std::string("hello/this").c_str(),
			 SysUtil::trimPath("hello/this/is/my", 2).c_str());
	    ASSERT_STREQ(std::string("is/my").c_str(),
			 SysUtil::trimPath("hello/this/is/my", 2, true).c_str());

	    ASSERT_STREQ(std::string("hello/this/is").c_str(),
			 SysUtil::trimPath("hello/this/is/my/file.txt", 2).c_str());
	    ASSERT_STREQ(std::string("is/my/file.txt").c_str(),
			 SysUtil::trimPath("hello/this/is/my/file.txt", 2, true).c_str());
	}

	TEST(sysutilTest, trimPath_updir) {
	    ASSERT_STREQ(std::string("../hello/this").c_str(),
			 SysUtil::trimPath("../hello/this/is/my", 2).c_str());
	    ASSERT_STREQ(std::string("this/is/my").c_str(),
			 SysUtil::trimPath("../hello/this/is/my", 2, true).c_str());

	    ASSERT_STREQ(std::string("../hello/this/is").c_str(),
			 SysUtil::trimPath("../hello/this/is/my/file.txt", 2).c_str());
	    ASSERT_STREQ(std::string("this/is/my/file.txt").c_str(),
			 SysUtil::trimPath("../hello/this/is/my/file.txt", 2, true).c_str());
	}

	TEST(sysutilTest, trimPath_currentdir) {
	    ASSERT_STREQ(std::string("./hello/this").c_str(),
			 SysUtil::trimPath("./hello/this/is/my", 2).c_str());
	    ASSERT_STREQ(std::string("this/is/my").c_str(),
			 SysUtil::trimPath("./hello/this/is/my", 2, true).c_str());

	    ASSERT_STREQ(std::string("./hello/this/is").c_str(),
			 SysUtil::trimPath("./hello/this/is/my/file.txt", 2).c_str());
	    ASSERT_STREQ(std::string("this/is/my/file.txt").c_str(),
			 SysUtil::trimPath("./hello/this/is/my/file.txt", 2, true).c_str());
	}

	TEST(sysutilTest, removePathBase_directory) {
	    ASSERT_STREQ(std::string("bin").c_str(),
			 SysUtil::removePathBase("/usr/bin/").c_str());
	}

	TEST(sysutilTest, removePathBase_file) {
	    ASSERT_STREQ(std::string("g++").c_str(),
			 SysUtil::removePathBase("/usr/bin/g++").c_str());
	}
    } // namespace sysutil
} // namespace testing

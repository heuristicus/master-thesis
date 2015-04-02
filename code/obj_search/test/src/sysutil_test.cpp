#include "sysutil/sysutil.hpp"
#include <gtest/gtest.h>

namespace testing {
    namespace sysutil {

	TEST(sysutilTest, cleanDirPath) {
	    ASSERT_STREQ("/test/the/function",
			 SysUtil::cleanDirPath("/test/the/function/").c_str());
	    ASSERT_STREQ("/test/the/function",
			 SysUtil::cleanDirPath("/test/the/function").c_str());
	}

	TEST(sysutilTest, fullDirPath) {
	    ASSERT_STREQ("/test/the/function/",
			 SysUtil::fullDirPath("/test/the/function").c_str());
	    ASSERT_STREQ("/test/the/function/",
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
	    ASSERT_STREQ("~/hello/this",
			 SysUtil::trimPath("~/hello/this/is/my", 2).c_str());
	    ASSERT_STREQ("~/hello/this/is",
			 SysUtil::trimPath("~/hello/this/is/my/file.txt", 2).c_str());

	}
	
	TEST(sysutilTest, trimPath_homedir_front) {
	    ASSERT_STREQ("this/is/my",
			 SysUtil::trimPath("~/hello/this/is/my", 2, true).c_str());
	    ASSERT_STREQ("this/is/my/file.txt",
			 SysUtil::trimPath("~/hello/this/is/my/file.txt", 2, true).c_str());
	}
	
	TEST(sysutilTest, trimPath_rootdir) {
	    ASSERT_STREQ("/hello/this",
			 SysUtil::trimPath("/hello/this/is/my", 2).c_str());
	    ASSERT_STREQ("/hello/this/is",
			 SysUtil::trimPath("/hello/this/is/my/file.txt", 2).c_str());
	}

	TEST(sysutilTest, trimPath_rootdir_front) {
	    ASSERT_STREQ("is/my",
			 SysUtil::trimPath("/hello/this/is/my", 2, true).c_str());
	    ASSERT_STREQ("is/my/file.txt",
			 SysUtil::trimPath("/hello/this/is/my/file.txt", 2, true).c_str());
	}
	

	TEST(sysutilTest, trimPath_noslash) {
	    ASSERT_STREQ("hello/this",
			 SysUtil::trimPath("hello/this/is/my", 2).c_str());
	    ASSERT_STREQ("hello/this/is",
			 SysUtil::trimPath("hello/this/is/my/file.txt", 2).c_str());
	}

	TEST(sysutilTest, trimPath_noslash_front) {
	    ASSERT_STREQ("is/my",
			 SysUtil::trimPath("hello/this/is/my", 2, true).c_str());
	    ASSERT_STREQ("is/my/file.txt",
			 SysUtil::trimPath("hello/this/is/my/file.txt", 2, true).c_str());
	}

	TEST(sysutilTest, trimPath_slash) {
	    ASSERT_STREQ("/hello/this",
			 SysUtil::trimPath("/hello/this/is/my", 2).c_str());
	    ASSERT_STREQ("/hello/this/is",
			 SysUtil::trimPath("/hello/this/is/my/file.txt", 2).c_str());
	}

	TEST(sysutilTest, trimPath_slash_front) {
	    ASSERT_STREQ("is/my",
			 SysUtil::trimPath("/hello/this/is/my", 2, true).c_str());
	    ASSERT_STREQ("is/my/file.txt",
			 SysUtil::trimPath("/hello/this/is/my/file.txt", 2, true).c_str());
	}

	
	TEST(sysutilTest, trimPath_updir) {
	    ASSERT_STREQ("../hello/this",
			 SysUtil::trimPath("../hello/this/is/my", 2).c_str());
	    ASSERT_STREQ("../hello/this/is",
			 SysUtil::trimPath("../hello/this/is/my/file.txt", 2).c_str());
	}

	TEST(sysutilTest, trimPath_updir_front) {
	    ASSERT_STREQ("this/is/my",
			 SysUtil::trimPath("../hello/this/is/my", 2, true).c_str());
	    ASSERT_STREQ("this/is/my/file.txt",
			 SysUtil::trimPath("../hello/this/is/my/file.txt", 2, true).c_str());
	}


	TEST(sysutilTest, trimPath_currentdir) {
	    ASSERT_STREQ("./hello/this",
			 SysUtil::trimPath("./hello/this/is/my", 2).c_str());
	    ASSERT_STREQ("./hello/this/is",
			 SysUtil::trimPath("./hello/this/is/my/file.txt", 2).c_str());
	}

	TEST(sysutilTest, trimPath_currentdir_front) {
	    ASSERT_STREQ("this/is/my",
			 SysUtil::trimPath("./hello/this/is/my", 2, true).c_str());
	    ASSERT_STREQ("this/is/my/file.txt",
			 SysUtil::trimPath("./hello/this/is/my/file.txt", 2, true).c_str());
	}


	TEST(sysutilTest, trimPath_reverse) {
	    ASSERT_STREQ("my",
			 SysUtil::trimPath("./hello/this/is/my", -1).c_str());
	    ASSERT_STREQ("is/my/file.txt",
			 SysUtil::trimPath("./hello/this/is/my/file.txt", -3).c_str());
	}

	TEST(sysutilTest, trimPath_reverse_front) {
	    ASSERT_STREQ(".",
			 SysUtil::trimPath("./hello/this/is/my", -1, true).c_str());
	    ASSERT_STREQ("./hello/this",
			 SysUtil::trimPath("./hello/this/is/my/file.txt", -3, true).c_str());
	    ASSERT_STREQ("~/hello/this",
			 SysUtil::trimPath("~/hello/this/is/my/file.txt", -3, true).c_str());
	    ASSERT_STREQ("../hello/this",
			 SysUtil::trimPath("../hello/this/is/my/file.txt", -3, true).c_str());
	    ASSERT_STREQ("/hello/this",
			 SysUtil::trimPath("/hello/this/is/my/file.txt", -3, true).c_str());
	    
	}

	TEST(sysutilTest, trimPath_edge_notrim) {
	    ASSERT_STREQ("./hello/this/is/my",
			 SysUtil::trimPath("./hello/this/is/my", 0).c_str());
	}

	TEST(sysutilTest, trimPath_edge_single) {
	    ASSERT_STREQ("usr",
			 SysUtil::trimPath("/usr", -1).c_str());
	}

	TEST(sysutilTest, trimPath_edge_toofar) {
	    ASSERT_STREQ("",
			 SysUtil::trimPath("./hello/this/is/my", 10).c_str());
	    ASSERT_STREQ("",
			 SysUtil::trimPath("./hello/this/is/my", 10, true).c_str());
	    ASSERT_STREQ("./hello/this/is/my",
			 SysUtil::trimPath("./hello/this/is/my", -10).c_str());
	    ASSERT_STREQ("./hello/this/is/my",
			 SysUtil::trimPath("./hello/this/is/my", -10, true).c_str());
	    ASSERT_STREQ("/hello/this/is/my",
			 SysUtil::trimPath("/hello/this/is/my", -10).c_str());
	    ASSERT_STREQ("/hello/this/is/my",
			 SysUtil::trimPath("/hello/this/is/my", -10, true).c_str());

	}

	TEST(sysutilTest, removeExtension_basic) {
	    ASSERT_STREQ("testfile",
			 SysUtil::removeExtension("testfile.txt").c_str());
	}

	TEST(sysutilTest, removeExtension_path) {
	    ASSERT_STREQ("testfile",
			 SysUtil::removeExtension("/some/random/path/testfile.txt").c_str());
	}

	TEST(sysutilTest, removeExtension_edge_noextension) {
	    ASSERT_STREQ("testfile",
			 SysUtil::removeExtension("/some/random/path/testfile").c_str());
	}

	TEST(sysutilTest, removePathBase_directory) {
	    ASSERT_STREQ("bin",
			 SysUtil::removePathBase("/usr/bin/").c_str());
	}

	TEST(sysutilTest, removePathBase_file) {
	    ASSERT_STREQ("g++",
			 SysUtil::removePathBase("/usr/bin/g++").c_str());
	}

	TEST(sysutilTest, combinePaths_filesBasic) {
	    std::string result = SysUtil::combinePaths("/usr/share/texlive/index.html",
						       "/usr/bin/more");
	    ASSERT_STREQ("/usr/share/texlive/usr/bin/more", result.c_str());
	}
	TEST(sysutilTest, combinePaths_filesUpdir) {
	    std::string result = SysUtil::combinePaths("/usr/share/texlive/index.html",
					   "../usr/bin/more");
	    ASSERT_STREQ("/usr/share/texlive/usr/bin/more", result.c_str());
	}
	
	TEST(sysutilTest, combinePaths_filesCurDir) {
	    std::string result = SysUtil::combinePaths("/usr/share/texlive/index.html",
					   "./usr/bin/more");
	    ASSERT_STREQ("/usr/share/texlive/usr/bin/more", result.c_str());
	}
	
	TEST(sysutilTest, combinePaths_filesHome) {
	    std::string result = SysUtil::combinePaths("/usr/share/texlive/index.html",
					   "~/usr/bin/more");
	    ASSERT_STREQ("/usr/share/texlive/usr/bin/more", result.c_str());
	}

	TEST(sysutilTest, combinePaths_filesNoslash) {
	    std::string result = SysUtil::combinePaths("/usr/share/texlive/index.html",
					   "usr/bin/more");
	    ASSERT_STREQ("/usr/share/texlive/usr/bin/more", result.c_str());
	}

	TEST(sysutilTest, combinePaths_directories) {
	    std::string result = SysUtil::combinePaths("/usr/share/texlive/",
						       "/usr/bin/");
	    ASSERT_STREQ("/usr/share/texlive/usr/bin", result.c_str());
	}

	TEST(sysutilTest, combinePaths_empty) {
	    std::string result = SysUtil::combinePaths("/usr/share/texlive/", "");
	    ASSERT_STREQ("/usr/share/texlive/", result.c_str());
	    result = SysUtil::combinePaths("", "/usr/share/texlive/");
	    ASSERT_STREQ("/usr/share/texlive/", result.c_str());
	    result = SysUtil::combinePaths("", "");
	    ASSERT_STREQ("", result.c_str());
	}
	

	TEST(sysutilTest, makeDirs_single){
	    std::string testDir("/tmp/ob_search_testdir897913823");
	    ASSERT_TRUE(SysUtil::makeDirs(testDir));
	    ASSERT_TRUE(SysUtil::isDir(testDir));
	    rmdir(testDir.c_str());
	}

	TEST(sysutilTest, makeDirs_single_existing){
	    ASSERT_TRUE(SysUtil::makeDirs("/tmp"));
	    ASSERT_TRUE(SysUtil::makeDirs("/usr/lib"));
	}

	TEST(sysutilTest, makeDirs_multiple){
	    std::string testDir("/tmp/ob_search/tstdir/item/chain");
	    ASSERT_TRUE(SysUtil::makeDirs(testDir));
	    ASSERT_TRUE(SysUtil::isDir("/tmp/ob_search"));
	    ASSERT_TRUE(SysUtil::isDir("/tmp/ob_search/tstdir"));
	    ASSERT_TRUE(SysUtil::isDir("/tmp/ob_search/tstdir/item"));
	    ASSERT_TRUE(SysUtil::isDir("/tmp/ob_search/tstdir/item/chain"));
	    rmdir("/tmp/ob_search/tstdir/item/chain");
	    rmdir("/tmp/ob_search/tstdir/item");
	    rmdir("/tmp/ob_search/tstdir");
	    rmdir("/tmp/ob_search");
	}

    } // namespace sysutil
} // namespace testing

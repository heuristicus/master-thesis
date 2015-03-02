/**
 * @file   tests.cpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Mon Mar  2 12:10:08 2015
 * 
 * @brief  gtest test code
 * 
 * Taken from
 * http://www.thebigblob.com/getting-started-with-google-test-on-ubuntu/
 */


#include "totest.hpp"
#include <gtest/gtest.h>
 
/** 
 * Test the positive numbers
 * 
 * @param SquareRootTest 
 * @param PositiveNos 
 * 
 * @return 
 */
TEST(SquareRootTest, PositiveNos) { 
    ASSERT_EQ(6, squareRoot(36.0));
    ASSERT_EQ(18.0, squareRoot(324.0));
    ASSERT_EQ(25.4, squareRoot(645.16));
    ASSERT_EQ(0, squareRoot(0.0));
}
 
/** 
 * Test the negative numbers
 * 
 * @param SquareRootTest 
 * @param NegativeNos 
 * 
 * @return 
 */
TEST(SquareRootTest, NegativeNos) {
    ASSERT_EQ(-1.0, squareRoot(-15.0));
    ASSERT_EQ(-1.0, squareRoot(-0.2));
}
 
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

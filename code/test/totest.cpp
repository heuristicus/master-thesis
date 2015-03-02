/**
 * @file   totest.cpp
 * @author Michal Staniaszek <michalst@kth.se>
 * @date   Mon Mar  2 10:42:51 2015
 * 
 * @brief  test brief description 
 *
 * I suppose this is the long description where you write stuff down?
 * \f[\sum_{i=1}^5\f]
 *
 * Test code from 
 * http://www.thebigblob.com/getting-started-with-google-test-on-ubuntu/
 * 
 */
#include "totest.hpp"
 
/** 
 * Square root function
 * 
 * @param a The thing to take the root of
 * 
 * @return The root of the thing
 */
double squareRoot(const double a) {
    double b = sqrt(a);
    if(b != b) { // nan check
        return -1.0;
    }else{
        return sqrt(a);
    }
}

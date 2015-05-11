#ifndef POINT_VALIDATION_H
#define POINT_VALIDATION_H

#include <pcl/features/shot.h>
#include <pcl/features/usc.h>
#include <cmath>

namespace objsearch {
    namespace pclutil {

	bool isValid(const pcl::SHOT352& desc)	{
	    return !std::isnan(desc.rf[0]);
	}

	bool isValid(const pcl::SHOT1344& desc) {
	    return !std::isnan(desc.rf[0]);
	}
	
	bool isValid(const pcl::PFHSignature125& desc) {
	    return true;
	}

	bool isValid(const pcl::FPFHSignature33& desc) {
	    return true;
	}

	bool isValid(const pcl::PFHRGBSignature250& desc) {
	    return true;
	}

	bool isValid(const pcl::ShapeContext1980& desc) {
	    return true;
	}
	
    } // namespace pclutil
} // namespace objsearch

#endif // POINT_VALIDATION_H



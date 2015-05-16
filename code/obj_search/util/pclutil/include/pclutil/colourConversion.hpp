#ifndef COLOUR_CONVERSION_HPP
#define COLOUR_CONVERSION_HPP

#include <cmath>

// http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
namespace objsearch {
    namespace pclutil {
	struct rgb {
	    rgb(){}
	    rgb(double _r, double _g, double _b) : r(_r), g(_g), b(_b){}
	    double r;       // percent
	    double g;       // percent
	    double b;       // percent
	};

	struct hsv {
	    hsv(){}
	    hsv(double _h, double _s, double _v) : h(_h), s(_s), v(_v){}
	    double h;       // angle in degrees
	    double s;       // percent
	    double v;       // percent
	};

	hsv rgb2hsv(rgb in) {
	    hsv         out;
	    double      min, max, delta;

	    min = in.r < in.g ? in.r : in.g;
	    min = min  < in.b ? min  : in.b;

	    max = in.r > in.g ? in.r : in.g;
	    max = max  > in.b ? max  : in.b;

	    out.v = max;                                // v
	    delta = max - min;
	    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
		out.s = (delta / max);                  // s
	    } else {
		// if max is 0, then r = g = b = 0              
		// s = 0, v is undefined
		out.s = 0.0;
		out.h = std::nan("");                            // its now undefined
		return out;
	    }
	    if( in.r >= max )                           // > is bogus, just keeps compilor happy
		out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
	    else
		if( in.g >= max )
		    out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
		else
		    out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

	    out.h *= 60.0;                              // degrees

	    if( out.h < 0.0 )
		out.h += 360.0;

	    return out;
	}


	rgb hsv2rgb(hsv in) {
	    double      hh, p, q, t, ff;
	    long        i;
	    rgb         out;

	    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
		out.r = in.v;
		out.g = in.v;
		out.b = in.v;
		return out;
	    }
	    hh = in.h;
	    if(hh >= 360.0) hh = 0.0;
	    hh /= 60.0;
	    i = (long)hh;
	    ff = hh - i;
	    p = in.v * (1.0 - in.s);
	    q = in.v * (1.0 - (in.s * ff));
	    t = in.v * (1.0 - (in.s * (1.0 - ff)));

	    switch(i) {
	    case 0:
		out.r = in.v;
		out.g = t;
		out.b = p;
		break;
	    case 1:
		out.r = q;
		out.g = in.v;
		out.b = p;
		break;
	    case 2:
		out.r = p;
		out.g = in.v;
		out.b = t;
		break;

	    case 3:
		out.r = p;
		out.g = q;
		out.b = in.v;
		break;
	    case 4:
		out.r = t;
		out.g = p;
		out.b = in.v;
		break;
	    case 5:
	    default:
		out.r = in.v;
		out.g = p;
		out.b = q;
		break;
	    }
	    return out;     
	}

	/** 
	 * Get the heat colour of a given value which is assumed to be from a
	 * range 0-maxValue. 0 is blue and maxValue is red.
	 *  
	 * @param value the value of the item to assign a heat colour to
	 * @param maxValue the max value of the set which contains \p value
	 * 
	 * @return 
	 */
	rgb getHeatColour(float value, float maxValue) {
	    static const float minHue = 0;
	    static const float maxHue = 247.5;
	    float hue = maxHue - ((float) (value / maxValue) * (maxHue - minHue));
	    return hsv2rgb(hsv(hue, 100, 100));
	}

	/** 
	 * Convert RGB values to intensity values. Assumes each channel has the
	 * same weighting.
	 * 
	 * @return 
	 */
	float getRGBIntensityBasic(int r, int g, int b) {
	    return (r + g + b) / 3;
	}
    } // namespace pclutil
} // namespace objsearch

#endif // COLOUR_CONVERSION_HPP

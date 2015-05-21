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

	/** 
	 * https://github.com/ratkins/RGBConverter/blob/master/RGBConverter.cpp
	 * 
	 * @param in 
	 * 
	 * @return 
	 */
	rgb hsv2rgb(hsv in) {
	    double r = 0, g = 0, b = 0;
	    double h = in.h;
	    double s = in.s;
	    double v = in.v;
	    

	    int i = int(h * 6);
	    double f = h * 6 - i;
	    double p = v * (1 - s);
	    double q = v * (1 - f * s);
	    double t = v * (1 - (1 - f) * s);

	    switch(i % 6){
	    case 0: r = v, g = t, b = p; break;
	    case 1: r = q, g = v, b = p; break;
	    case 2: r = p, g = v, b = t; break;
	    case 3: r = p, g = q, b = v; break;
	    case 4: r = t, g = p, b = v; break;
	    case 5: r = v, g = p, b = q; break;
	    }

	    return rgb(r, g, b);
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
	    static const float minHue = 0; // red
	    static const float maxHue = 0.7; // blue
	    float hue = maxHue - ((float) (value / maxValue) * (maxHue - minHue));
	    return hsv2rgb(hsv(hue, 1, 1));
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

	/** 
	 * Convert a hex representation of RGB colour to its rgb representation.
	 * 
	 * @param hex hex colour in the form 0xRRGGBB
	 * @param r 
	 * @param g 
	 * @param b 
	 */
	void hexToRGB(int hex, int& r, int& g, int& b) {
	    r = ((hex >> 16) & 0xFF);  // Extract the RR byte
	    g = ((hex >> 8) & 0xFF);   // Extract the GG byte
	    b = ((hex) & 0xFF);        // Extract the BB byte
	}
	
    } // namespace pclutil
} // namespace objsearch

#endif // COLOUR_CONVERSION_HPP

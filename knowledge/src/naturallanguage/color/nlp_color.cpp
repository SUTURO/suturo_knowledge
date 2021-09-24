#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <boost/math/distributions/normal.hpp>
    using boost::math::normal; // typedef provides default type is double.




/*** Rgb2Hsv
* converts RGB color values to Hues/HSV Values
* taken from https://www.programmersought.com/article/46686765063/
*/

void Rgb2Hsv(double R, double G, double B, double& H, double& S, double& V)
{
	// r,g,b values are from 0 to 1
	// h = [0,360], s = [0,1], v = [0,1]
	// if s == 0, then h = -1 (undefined)
	double min, max, delta, tmp;
	tmp = R>G ? G : R;
	min = tmp>B ? B : tmp;
	tmp = R>G ? R : G;
	max = tmp>B ? tmp : B;
	V = max / 255 * 100; // v
	delta = max - min;
	if (max != 0)
		S = delta / max *100; // s
	else
	{
		// r = g = b = 0 // s = 0, v is undefined
		S = 0;
		H = 0;
		return;
	}
	if (delta == 0) 
	{
		H = 0;
		return;
	}
	else if (R == max)
	{
		if (G >= B)
			H = (G - B) / delta; // between yellow & magenta
		else
			H = (G - B) / delta + 6.0;
	}
	else if (G == max)
		H = 2.0 + (B - R) / delta; // between cyan & yellow
	else if (B == max)
		H = 4.0 + (R - G) / delta; // between magenta & cyan
	
	H *= 60.0; // degrees
	
}

// r,g,b values are from 0 to 1
// h = [0,360], s = [0,1], v = [0,1]
// if s == 0, then h = -1 (undefined)
// + [float, float, float], - [float,float, float]
PREDICATE(rgb_to_hsv,2) {
    double R ,G ,B, H, S, V;

    PlTail list(PL_A1);
    PlTerm value;
    list.next(value); R = (double) value;
	list.next(value); G = (double) value;
	list.next(value); B = (double) value;

    Rgb2Hsv(R, G, B, H, S, V);

	PlTail l(PL_A2);
	l.append(H);
	l.append(S);
	l.append(V);
	l.close();

    return TRUE;
}




// *****************************************************************************************



/** + value list of 3 elements
    +mean parameter list of 3,
    +standard deviation of 3
    +multipliers list of 3
    -propability one float
*/
PREDICATE(three_normal_distribution_propability_multi,5) {

    /*
    Convert the Inputs into usable lists of Terms
    */

    PlTail values(PL_A1);       PlTerm value;
    PlTail means(PL_A2);        PlTerm mean;
    PlTail std_devs(PL_A3);     PlTerm std_dev;
    PlTail multipliers(PL_A4);  PlTerm multi;


    double return_value = 1; // we will convert this into a Term later


    for ( int i = 0; i < 3 ; i++) {
        // let the PlTerms be the next/first element of the PlTails
        values.next(value);
        means.next(mean);
        std_devs.next(std_dev);
        multipliers.next(multi);

        // we have to cast the PlTerms to double
        return_value *= 
        pdf( 
            normal((double) mean, (double) std_dev), // create a normal dist
            (double) value
        ) * (double) multi;


    }


    PL_A5 = PlTerm(return_value);

    return TRUE;
}

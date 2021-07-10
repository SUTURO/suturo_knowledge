#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <boost/math/distributions/normal.hpp> // for normal_distribution
  using boost::math::normal; // typedef provides default type is double.
#include <iomanip>
  using std::setw; using std::setprecision;
#include <limits>
  using std::numeric_limits;



using namespace std;


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

// + mean, +standard_deviation, +z, -propability
PREDICATE(normal_distribution_propability,4) {


    // Construct a normal distribution s
    normal s((double) PL_A1, (double) PL_A2);

    PL_A4 = PlTerm(pdf(s, (double) PL_A3));


    return TRUE;
}



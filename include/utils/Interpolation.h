#ifndef __INTERPOLATION_H__
#define __INTERPOLATION_H__

#include <cmath>

namespace bipedal
{

/*
 * Linear interpolation of S in period time T
 */
inline double linearInterpolation(double t, double T, double S)
{
	return (t/T) * S;
}

inline double spline(double t, double x1, double x2, double y1, double y2, double k1, double k2)
{
	const double dx = (x2-x1);
	const double dy = (y2-y1);
	const double a =  k1 * dx - dy;
	const double b = -k2 * dx + dy;
	return ((1-t) * y1 + t * y2 + t*(1-t)*(a*(1-t) + b*t));
}

inline double splineDerivation(double t, double x1, double x2, double y1, double y2, double k1, double k2)
{
	const double dx = (x2-x1);
	const double dy = (y2-y1);
	const double a =  k1 * dx - dy;
	const double b = -k2 * dx + dy;
	return dy/dx + (1-2*t) * (a*(1-t) + b*t)/dx + t*(1-t)*(b - a)/dx;
}

/*
 * Cubic spline interpolation of S in period time T.
 * This should give constant jerk and linear accelerations.
 */
inline double splinePositionInterpolation(double t, double T, double S, double v0, double v1)
{
	double y = spline(t/T, 0, T, 0, S, v0, v1);
	return y;
}

inline double splineVelocityInterpolation(double t, double T, double S, double v0, double v1)
{
	double y = splineDerivation(t/T, 0, T, 0, S, v0, v1);
	return y;
}

}

#endif

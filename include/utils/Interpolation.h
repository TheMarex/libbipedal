/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __INTERPOLATION_H__
#define __INTERPOLATION_H__

#include <cmath>

namespace Bipedal
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

/**
 * Derived from a polynom with degree 5 using position/velocity/acceleration constrains for the end-points
 */
inline double polyPositionInterpolation(double t, double T, double S, double v0, double v1, double a0, double a1)
{
	const double x = t;
	return (x*(pow(T, 5)*(a0*x + 2*v0) + pow(T, 2)*pow(x, 2)*(20*S + pow(T, 2)*(-3*a0 + a1) - 4*T*(3*v0 + 2*v1)) + T*pow(x, 3)*(-30*S + pow(T, 2)*(3*a0 - 2*a1) + 2*T*(8*v0 + 7*v1)) + pow(x, 4)*(12*S + pow(T, 2)*(-a0 + a1) - 6*T*(v0 + v1)))/(2*pow(T, 5)));
}

/**
 * Derived from a polynom with degree 5 using position/velocity/acceleration constrains for the end-points
 */
inline double polyVelocityInterpolation(double t, double T, double S, double v0, double v1, double a0, double a1)
{
	const double x = t;
	return (a0*x + v0 + pow(x, 4)*(30*S/pow(x, 5) - 5*a0/(2*pow(x, 3)) + 5*a1/(2*pow(x, 3)) - 15*v0/pow(x, 4) - 15*v1/pow(x, 4)) + pow(x, 3)*(-60*S/pow(x, 4) + 6*a0/pow(x, 2) - 4*a1/pow(x, 2) + 32*v0/pow(x, 3) + 28*v1/pow(x, 3)) + pow(x, 2)*(30*S/pow(x, 3) - 9*a0/(2*T) + 3*a1/(2*T) - 18*v0/pow(x, 2) - 12*v1/pow(x, 2)));
}

template<typename VectorT>
class CubivBezierCurve
{
public:
    VectorT position;
    VectorT start;
    VectorT end;
    VectorT h1;
    VectorT h2;

    CubivBezierCurve()
    : position(VectorT::Zero())
    , start(VectorT::Zero())
    , end(VectorT::Zero())
    , h1(VectorT::Zero())
    , h2(VectorT::Zero())
    , T(0)
    , t(0)
    {
    }

    CubivBezierCurve(const VectorT& start, const VectorT& end, const VectorT& h1, const VectorT& h2, double T)
    : start(start)
    , end(end)
    , h1(h1)
    , h2(h2)
    , T(T)
    , t(0)
    , position(VectorT::Zero())
    {
    }

    inline bool finished() { return t > T; }

    /**
     * Use a spline for time.
     * This makes sure we have zero velocity at start and end.
     */
    inline double getTime() const
    {
        const double x = t/T;
        return x*x*x*(6.0*x*x - 15.0*x + 10.0);
    }

    /**
     * dt is the timestep size.
     */
    inline void update(double dt)
    {
        const double x = getTime();
        const double s = 1-x;
        t += dt;
        position = s*s*s * start + 3*s*s*x * h1 + 3*s*x*x * h2 + x*x*x * end;
    }

private:
    double T;
    double t;
};

struct StepInterpolation
{
public:
    Eigen::Vector3f position;
    Eigen::Vector3f start;
    Eigen::Vector3f end;
private:
    double t;
    double T;
    double cx;
    double dx;
    double bz;
    double cz;
    double dz;

public:
    StepInterpolation()
    : position(Eigen::Vector3f::Zero())
    , start(Eigen::Vector3f::Zero())
    , end(Eigen::Vector3f::Zero())
    , T(0)
    , t(0)
    {
    }

    StepInterpolation(const Eigen::Vector3f& start, const Eigen::Vector3f& end, double stepHeight, double T)
    : T(T)
    , start(start)
    , end(end)
    {
        cx = -2 / (T*T*T);
        dx =  3 / (T*T);
        bz = 16  * stepHeight / (T*T*T*T);
        cz = -32 * stepHeight / (T*T*T);
        dz = 16  * stepHeight / (T*T);
    }

    inline bool finished() { return t > T; }

    inline void update(double dt)
    {
        double x1, x2, x3, x4;
        x1 = t;
        x2 = x1 * x1;
        x3 = x2 * x1;
        x4 = x3 * x1;

        double s = cx * x3 + dx * x2;
        double z = bz * x4 + cz * x3 + dz * x2;

        position = (1-s) * start + s * end;
        position.z() = z;

        t += dt;
    }
};

using CubivBezierCurve3f = CubivBezierCurve<Eigen::Vector3f>;
using CubivBezierCurve2f = CubivBezierCurve<Eigen::Vector2f>;

}

#endif

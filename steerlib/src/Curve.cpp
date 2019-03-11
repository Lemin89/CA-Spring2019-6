//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include <iostream>
#include "Globals.h"

using namespace Util;
using namespace std;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	Point startPoint = controlPoints.at(0).position;
	Point endPoint = controlPoints.at(0).position;
	int nextPoint = 0;
	float normalTime, intervalTime;
	float f1, f2, f3, f4;

	if (type == hermiteCurve)
	{
		//cout << "***hermiteCurve" << endl;
		for (int i = window; i <= controlPoints.at(controlPoints.size() - 1).time; i = i + window) {
			startPoint = endPoint;
			if (i > controlPoints.at(nextPoint).time)
				nextPoint++;
			if (i == window) {
				nextPoint = 1;
			}
			intervalTime = controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time;
			normalTime = (i - controlPoints.at(nextPoint - 1).time) / intervalTime;

			f1 = 2 * pow(normalTime, 3) - 3 * pow(normalTime, 2) + 1;
			f2 = -2 * pow(normalTime, 3) + 3 * pow(normalTime, 2);
			f3 = pow(normalTime, 3) - 2 * pow(normalTime, 2) + normalTime;
			f4 = pow(normalTime, 3) - pow(normalTime, 2);

			endPoint = Point(f1*controlPoints.at(nextPoint - 1).position + f2*controlPoints.at(nextPoint).position + f3*controlPoints.at(nextPoint - 1).tangent*intervalTime + f4*controlPoints.at(nextPoint).tangent*intervalTime);
			Util::DrawLib::drawLine(startPoint, endPoint, curveColor, curveThickness);
		}
	}else if (type == catmullCurve) 
	{
		//cout << "***catmullCurve" << endl;
		for (int i = 1; i < controlPoints.size() - 1;i++) {
			controlPoints.at(i).tangent = (controlPoints.at(i + 1).position - controlPoints.at(i - 1).position) / 2;
		}
		for (int i = window; i <= controlPoints.at(controlPoints.size() - 1).time; i = i + window) {
			startPoint = endPoint;
			if (i > controlPoints.at(nextPoint).time)
				nextPoint++;
			if (i == window) {
				nextPoint = 1;
			}
			if (nextPoint == controlPoints.size()) {
				nextPoint = controlPoints.size() - 1;
			}
			intervalTime = controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time;
			normalTime = (i - controlPoints.at(nextPoint - 1).time) / intervalTime;

			f1 = 2 * pow(normalTime, 3) - 3 * pow(normalTime, 2) + 1;
			f2 = -2 * pow(normalTime, 3) + 3 * pow(normalTime, 2);
			f3 = pow(normalTime, 3) - 2 * pow(normalTime, 2) + normalTime;
			f4 = pow(normalTime, 3) - pow(normalTime, 2);

			endPoint = Point(f1*controlPoints.at(nextPoint - 1).position + f2*controlPoints.at(nextPoint).position + f3*controlPoints.at(nextPoint - 1).tangent*intervalTime + f4*controlPoints.at(nextPoint).tangent*intervalTime);
			Util::DrawLib::drawLine(startPoint, endPoint, curveColor, curveThickness);
		}
	}
	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve
	
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{	
	for (int a = 0; a < controlPoints.size(); a = a + 1) {
		for (int b = a + 1; b < controlPoints.size(); b = b + 1) {
			if (controlPoints.at(a).time > controlPoints.at(b).time) {
				CurvePoint temp1 = controlPoints.at(a);
				controlPoints.at(a) = controlPoints.at(b);
				controlPoints.at(b) = temp1;
			}
		}
	}
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if (controlPoints.size() < 2)
		return false;
	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{	
	nextPoint = 0;
	for (int i = 0; i < controlPoints.size();i++) {
		if (time >= controlPoints.at(nextPoint).time)
			nextPoint++;
	}
	if (nextPoint >= 0 && nextPoint < controlPoints.size() && controlPoints.at(nextPoint).time > time) {
		return true;
	}
	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;
	intervalTime = controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time;
	normalTime = (time - controlPoints.at(nextPoint - 1).time) / intervalTime;
	float f1 = 2 * pow(normalTime, 3) - 3 * pow(normalTime, 2) + 1;
	float f2 = -2 * pow(normalTime, 3) + 3 * pow(normalTime, 2);
	float f3 = pow(normalTime, 3) - 2 * pow(normalTime, 2) + normalTime;
	float f4 = pow(normalTime, 3) - pow(normalTime, 2);
	newPosition = Point(f1*controlPoints.at(nextPoint - 1).position + f2*controlPoints.at(nextPoint).position + f3*controlPoints.at(nextPoint - 1).tangent*intervalTime + f4*controlPoints.at(nextPoint).tangent*intervalTime);
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	//update tangent of control points
	if (nextPoint>0&& nextPoint<controlPoints.size()-1) {
		controlPoints.at(nextPoint).tangent = (controlPoints.at(nextPoint + 1).position - controlPoints.at(nextPoint - 1).position) / 2;
	}
	Point newPosition;
	float normalTime, intervalTime;
	intervalTime = controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time;
	normalTime = (time - controlPoints.at(nextPoint - 1).time) / intervalTime;
	float f1 = 2*pow(normalTime, 3) - 3 * pow(normalTime, 2) + 1;
	float f2 = -2 * pow(normalTime, 3) + 3 * pow(normalTime, 2);
	float f3 = pow(normalTime, 3) - 2 * pow(normalTime, 2) + normalTime;
	float f4 = pow(normalTime, 3) - pow(normalTime, 2);
	newPosition = Point(f1*controlPoints.at(nextPoint - 1).position + f2*controlPoints.at(nextPoint).position + f3*controlPoints.at(nextPoint - 1).tangent*intervalTime + f4*controlPoints.at(nextPoint).tangent*intervalTime);
	return newPosition;
}

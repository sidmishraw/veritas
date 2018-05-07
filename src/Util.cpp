
// Kevin M.Smith - CS 134 SJSU

#include "Util.h"

#include <iostream>
using namespace std;

//---------------------------------------------------------------
// test if a ray intersects a plane.  If there is an intersection,
// return true and put point of intersection in "point"
//
bool rayIntersectPlane(const ofVec3f &rayPoint, const ofVec3f &raydir, const ofVec3f &planePoint,
                       const ofVec3f &planeNorm, ofVec3f &point) {
  // if d1 is 0, then the ray is on the plane or there is no intersection
  //
  const float eps = .000000001;
  float d1 = (planePoint - rayPoint).dot(planeNorm);
  if (abs(d1) < eps) return false;

  //  if d2 is 0, then the ray is parallel to the plane
  //
  float d2 = raydir.dot(planeNorm);
  if (abs(d2) < eps) return false;

  //  compute the intersection point and return it in "point"
  //
  point = (d1 / d2) * raydir + rayPoint;
  return true;
}

// Compute the reflection of a vector incident on a surface at the normal.
//
//
ofVec3f reflectVector(const ofVec3f &v, const ofVec3f &n) { return (v - 2 * v.dot(n) * n); }

// Added by sidmishraw -----
// Minimizes the vectors passed in and returns the vector with the minimum
// of all the x, y, z coordinates.
// For eg: if we pass in (1, 1, 0) and (1, 0, 1)
// the minimized vector would be (1, 0, 0).
//
Vector3 minimize(const Vector3 &a, const Vector3 &b) {
  return Vector3(a.x() <= b.x() ? a.x() : b.x(), a.y() <= b.y() ? a.y() : b.y(), a.z() <= b.z() ? a.z() : b.z());
}

// Added by sidmishraw -----
// Maximizes the vectors passed in and returns the vector with the maximum
// of all the x, y, z coordinates.
// For eg: if we pass in (1, 1, 0) and (1, 0, 1)
// the maximized vector would be (1, 1, 1).
//
Vector3 maximize(const Vector3 &a, const Vector3 &b) {
  return Vector3(a.x() >= b.x() ? a.x() : b.x(), a.y() >= b.y() ? a.y() : b.y(), a.z() >= b.z() ? a.z() : b.z());
}

// Added by sidmishraw ----
// Compose together Boxes to make a larger box bounding
// all the smaller boxes.
//
Box compose(Box &box1, Box &box2) {
  auto box1Min = box1.min();
  auto box1Max = box1.max();

  //    cout << "Box1 min = "
  //    << box1Min.x() << ", "
  //    << box1Min.y() << ", "
  //    << box1Min.z() << endl;

  //    cout << "Box1 max = "
  //    << box1Max.x() << ", "
  //    << box1Max.y() << ", "
  //    << box1Max.z() << endl;

  auto box2Min = box2.min();
  auto box2Max = box2.max();

  //    cout << "Box2 min = "
  //    << box2Min.x() << ", "
  //    << box2Min.y() << ", "
  //    << box2Min.z() << endl;

  //    cout << "Box2 max = "
  //    << box2Max.x() << ", "
  //    << box2Max.y() << ", "
  //    << box2Max.z() << endl;

  auto min = box1Min;
  auto max = box1Max;

  if (box2Max < min) min = box2Max;
  if (box2Min < min) min = box2Min;

  if (max < box2Min) max = box2Min;
  if (max < box2Max) max = box2Max;

  // auto min = minimize(box1Min, box2Min);
  // auto max = maximize(box1Max, box2Max);

  return Box(min, max);
}

void logBox(Box &box) {
  cout << "Box --------" << endl;
  cout << box.min().x() << ", " << box.min().y() << ", " << box.min().z() << endl;
  cout << box.max().x() << ", " << box.max().y() << ", " << box.max().z() << endl;
  cout << "------------" << endl;
}

void log(string msg, short int level) {
  switch (level) {
    case 0: {
      // debug mode
      msg = "DEBUG :: " + msg;
      break;
    }
    case 1: {
      // info mode
      msg = "INFO :: " + msg;
      break;
    }
    case 2: {
      // error mode
      msg = "ERROR :: " + msg;
      break;
    }
    default: {
      msg = "INFO :: " + msg;
      break;
    }
  }
  cout << msg << endl;
}

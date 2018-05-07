#pragma once
//  Kevin M. Smith - CS 134 SJSU

#include "ofMain.h"

#include "box.h"
#include "ray.h"

using namespace std;

bool rayIntersectPlane(const ofVec3f &rayPoint, const ofVec3f &raydir, ofVec3f const &planePoint,
                       const ofVec3f &planeNorm, ofVec3f &point);

ofVec3f reflectVector(const ofVec3f &v, const ofVec3f &normal);

// Added by sidmishraw ----->
//

// Compose together Boxes to make a larger box bounding
// all the smaller boxes.
//
Box compose(Box &box1, Box &box2);

// Logs the bounds of the Box.
//
void logBox(Box &box);

// -- added by sidmishraw
// Logs the message to the console.
//
void log(string msg, short int level = 0);

#pragma once

#include "box.h"
#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include "ofxGui.h"
#include "ray.h"

#include "Util.h"
#include "octtree.h"

#include "Tmnper.hpp"  // for persistence -- by sidmishraw

using namespace sidmishraw_octtree;
using namespace std;

// The file name extension used for saving the path points
//
const string FILE_EXT = ".mars";

// The various app modes.
//
enum AppMode {
  NORMAL,                         // normal mode
  POINT_SELECTION_MODE,           // select the point to retarget the camera
  PATH_CREATION_MODE,             // create the path for the rover to follow
  PATH_EDIT_MODE,                 // edit the path the rover is following
  ROVER_ANIMATION_MODE,           // animate the rover -- move
  ANIMATION_BEGIN_SELECTION_MODE  // select the starting point for the animation
};

class ofApp : public ofBaseApp {
 public:
  void setup();
  void update();
  void draw();

  void keyPressed(int key);
  void keyReleased(int key);
  void mouseMoved(int x, int y);
  void mouseDragged(int x, int y, int button);
  void mousePressed(int x, int y, int button);
  void mouseReleased(int x, int y, int button);
  void mouseEntered(int x, int y);
  void mouseExited(int x, int y);
  void windowResized(int w, int h);
  void dragEvent(ofDragInfo dragInfo);
  void gotMessage(ofMessage msg);

  // ------- ours --------------------------------------------
  // Any attribute or operation by us will be introduced from
  // this point on. Only openframework specific code remains
  // above this point.
  //
  void drawAxis(ofVec3f);
  void initLightingAndMaterials();
  void savePicture();
  void toggleWireframeMode();
  void togglePointsDisplay();
  void toggleSelectTerrain();
  void setCameraTarget();
  bool doPointSelection();
  void drawBox(const Box &box);

  Box meshBounds(const ofMesh &);
  void subDivideBox8(const Box &b, vector<Box> &boxList);

  bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);

  // -- added by sidmishraw ---
  // Draws the bounding box around terrain
  //
  void drawBoundingBoxT();

  // -- added by sidmishraw ---
  // Draws the bounding box around rover
  //
  void drawBoundingBoxR();

  // -- added by sidmishraw --
  // Checks if the rover was selected with the mouse click by
  // checking if the ray intersects any of the bounding boxes of the
  // rover's components or the rover itself.
  //
  bool roverSelected(const ofVec3f &mousePoint);

  // -- added by sidmishraw
  // Switches the camera to view from.
  // (Camera index ranges from 0 - 4)
  //
  void switchCamera();

  // -- added by sidmishraw
  // Update the cameras' position and look at points, and targets
  //
  void updateCams();

  // ------ attrs ----------------------

  // -- added by sidmishraw --
  // cams[0] - world camera
  // cams[1] - driver's perspective camera
  // cams[2] - tracking camera
  // cams[3] - follow camera
  // cams[4] - rear facing camera
  //
  ofEasyCam cams[5];

  // flag denoting if the POV cam has panned
  //
  bool bPanned;

  // offsets the follow cam so that it can follow
  // the rover.
  //
  const float OFFSET_FOLLOW_CAM = 6.0f;

  // -- added by sidmishraw --
  // This index denotes which camera is selected
  //
  unsigned short int cameraIndex;

  ofxAssimpModelLoader mars, rover;
  ofLight light;

  ofVec3f selectedPoint;
  ofVec3f intersectPoint;
  const float selectionRange = 4.0;

  // boolean flags -- used for modes
  //
  bool bAltKeyDown;
  bool bCtrlKeyDown;

  bool bWireframe;
  bool bDisplayPoints;
  bool bPointSelected;  // flag if point selection mode is online
  bool bTerrainSelected;

  bool bRoverLoaded;
  bool bRoverSelected;

  // -- added by sidmishraw
  // Toggle velocity slider
  bool tglVelSlider;

  // -- added by sidmishraw
  //

  // -- added by sidmishraw --
  // boundingBoxT - bounding box for the terrain
  // boundingBoxR - bounding box for the rover
  // roverCBBoxes - the list of bounding boxes for each of rover's components
  //
  Box boundingBoxT, boundingBoxR;
  vector<Box> roverCBBoxes;

  // -- added by sidmishraw --
  // octtree for the terrain
  //
  shared_ptr<OctTree> octtreeT;

  // -- added by sidmishraw --
  // for debugging rover
  //
  vector<ofVec3f> pathPoints;

  // -- added by sidmishraw --
  // The path to animate against.
  //
  ofPolyline thePath;

  // --added by sidmishraw
  // The point where the mouse was clicked, transformed from
  // screen co-ordinate space to the world co-ordinate space.
  //
  ofVec3f mousePoint;

  // --- added by sidmishraw
  // the index of the next point on the path
  //
  unsigned int nextPtIndex;

  // -- added by sidmishraw
  // the percentage of path completed
  //
  float pct;

  // -added by sidmishraw
  // Gets the next percentage of path completed.
  //
  float nextPct();
  // Gets the prev percentage of path completed.
  //
  float prevPct();

  // For debugging --
  //
  ofVec3f roverPos;

  // -- added by sidmishraw
  // Moves the rover on the selected path, starting from the specified
  // point.
  //
  void moveRover();

  // -- added by sidmishraw
  // Starts playing the animation by moving the rover.
  //
  void playAnimation();

  // -- added by sidmishraw
  // Velocity magnitude slider.
  //
  ofxPanel gui;
  ofxFloatSlider velSlider;

  // -- added by sidmishraw
  // Added for deterministic mode of the app.
  //
  AppMode mode;

  // -- added by sidmishraw
  // Selected point index for modifying the path
  //
  int selectedPtIndex;

  // -- added by sidmishraw
  // This flag indicates if the mouse button is held down.
  //
  bool bMouseDown;

  // -- added by sidmishraw
  // Tracks the rover's current heading direction -- angle
  //
  float roverHeadingAngle;

  // Keeps track of rover's current orientation
  //
  ofVec3f roverOrientation;
  // Keeps track of rotations on the rover
  //
  int rotCount;

  // Animation related startpoint and index
  //
  ofVec3f aniStartPt;
  int aniSelectedIndex;

  // -- added by sidmishraw
  // Persistence to disk
  //
  // Loads the pathPoints vector from the string
  //
  void pathPointsFromString(string str);
  // Creates the string representation for the pathPoints vector
  //
  string pathPointsToString();
  // Save the path points to a file using the current timestamp
  //
  bool savePathToDisk();
  // Load the contents from disk given the file name
  //
  void loadPathFromDisk(string fileName);
  // Load the rover model from disk
  //
  void loadRoverModel(string filePath);
};

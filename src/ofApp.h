#pragma once

#include "box.h"
#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include "ray.h"

#include "Util.h"
#include "octtree.h"

using namespace sidmishraw_octtree;
using namespace std;
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

  bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm,
                           ofVec3f &point);

  // -- added by sidmishraw ---
  // Draws the bounding box around terrain
  //
  void drawBoundingBoxT();

  // -- added by sidmishraw ---
  // Draws the bounding box around rover
  //
  void drawBoundingBoxR();

  // cams[0] - world camera
  // cams[1] - driver's perspective camera
  // cams[2] - tracking camera
  // cams[3] - follow camera
  // cams[4] - rear facing camera
  //
  ofEasyCam cams[5];

  ofxAssimpModelLoader mars, rover;
  ofLight light;

  ofVec3f selectedPoint;
  ofVec3f intersectPoint;
  const float selectionRange = 4.0;

  // boolean flags
  //
  bool bAltKeyDown;
  bool bCtrlKeyDown;

  bool bWireframe;
  bool bDisplayPoints;
  bool bPointSelected;
  bool bTerrainSelected;

  bool bRoverLoaded;
  bool bRoverSelected;

  // This index denotes which camera is selected
  //
  unsigned short int cameraIndex;

  // boundingBoxT - bounding box for the terrain
  // boundingBoxR - bounding box for the rover
  Box boundingBoxT, boundingBoxR;

  // octtree for the terrain
  shared_ptr<OctTree> octtreeT;

  // for debugging rover
  vector<Box> boxes;
};

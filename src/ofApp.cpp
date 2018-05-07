
//--------------------------------------------------------------
//
//  Kevin M. Smith
//
//  Mars HiRise Project - startup scene
//
//  This is an openFrameworks 3D scene that includes an EasyCam
//  and example 3D geometry which I have reconstructed from Mars
//  HiRis photographs taken the Mars Reconnaisance Orbiter
//
//  You will use this source file (and include file) as a starting point
//  to implement assignment 5  (Parts I and II)
//
//  Please do not modify any of the keymappings.  I would like
//  the input interface to be the same for each student's
//  work.  Please also add your name/date below.

//  Please document/comment all of your work !
//  Have Fun !!
//
//  Student Name:   Sidharth Mishra <sidmishraw@gmail.com>
//  Date:           April 18, 2018

#include "ofApp.h"
#include "Util.h"
#include "ofxGui.h"

#include <algorithm>  // for algorithms
#include <chrono>     // for logging time taken for execution.

using namespace std;
using namespace sidmishraw_octtree;
using namespace std::chrono;

// added by sidmishraw ---
// Maximum allowable depth at the moment.
//
const int MAX_LEVEL = 5;

// added by sidmishraw ---
// Performs the initial setup for the 4 cameras
//
void setupCameras(ofEasyCam *cams) {
  for (int i = 0; i < 5; i++) {
    cams[i].setDistance(10);
    cams[i].setNearClip(.1);
    cams[i].setFov(65.5);
    ofSetVerticalSync(true);
    cams[i].disableMouseInput();
  }
}

//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//

void ofApp::setup() {
  bWireframe = false;
  bDisplayPoints = false;
  bAltKeyDown = false;
  bCtrlKeyDown = false;

  // -- added by sidmishraw
  //
  bAnimationOn = false;
  pct = 0;
  selectedPtIndex = -1;
  roverHeadingAngle = 0;  // 0 degrees

  tglVelSlider = false;

  // -- by default the terrain is deselected and rover is selected
  bTerrainSelected = false;

  bRoverLoaded = false;
  bRoverSelected = false;  // true;

  //	ofSetWindowShape(1024, 768);

  // camera setup
  //
  cameraIndex = 0;  // default camera set to world camera
  bPanned = false;

  setupCameras(cams);

  // setup rudimentary lighting
  //
  initLightingAndMaterials();

  mars.loadModel("geo/mars-low-v2.obj");
  mars.setScaleNormalization(false);

  boundingBoxT = meshBounds(mars.getMesh(0));

  cout << "Mesh count - terrain = " << mars.getMeshCount() << endl;

  // Octtree for terrain
  //
  octtreeT = make_shared<OctTree>();
  octtreeT->generate(mars.getMesh(0), MAX_LEVEL);

  // adding GUI slider
  //
  gui.setup();
  gui.add(velSlider.setup("Velocity", 0.1, 1.0, 5.0));
}

// -- added by sidmishraw
// Update the cameras
//
void ofApp::updateCams() {
  if (bRoverLoaded) {
    // Rover's position
    auto rpos = rover.getPosition();

    // rover's max bound's point - relative to rover
    // Since the scene max is in Rover's object space
    // I need to multiply it with the model matrix to bring it
    // into world space.
    //
    auto rmax = rover.getSceneMax() * rover.getModelMatrix();

    // Driver's view
    // POV of the rover's driver - front
    //
    cams[1].setPosition(ofVec3f(rpos.x, rmax.y, rpos.z));

    if (!bPanned) {
      // look forward
      cams[1].pan(180);
      bPanned = true;
    }

    // Tracking camera
    // tracks the rover from a fixed point -- highest point of the
    // terrain's bounding box.
    //
    auto trackingPoint = mars.getSceneMax();
    cams[2].setPosition(trackingPoint);
    cams[2].lookAt(rpos);     // tracking camera looks at the rover
    cams[2].setTarget(rpos);  // tracking camera orbits around the rover

    // Follow camera
    // follows the rover at a fixed
    //
    // follow camera following the rover from offset
    cams[3].setPosition(rpos.x - 3.0, rpos.y + OFFSET_FOLLOW_CAM, rpos.z - 3.0);
    cams[3].lookAt(rpos);     // always keep looking at the rover
    cams[3].setTarget(rpos);  // always orbit around the rover

    // Rear view
    // POV of the rover's driver - rear
    // By default the camera looks back --
    //
    cams[4].setPosition(ofVec3f(rpos.x, rmax.y, rpos.z));
    cams[4].pan(0);  // always look back -- rear
  }
}

// -- added by sidmishraw
// Move the rover along the path
//
void ofApp::moveRover() {
  if (!bAnimationOn) return;
  if (nextPtIndex == pathPoints.size()) nextPtIndex = 0;

  // rover's current and next position
  //
  auto p = rover.getPosition();                    // current posn
  auto roverPos = thePath.getPointAtPercent(pct);  // next posn

  // Set the new position of the rover along the path
  //
  rover.setPosition(roverPos.x, roverPos.y, roverPos.z);

  auto delta = (p * rover.getModelMatrix().getInverse()).angle(roverPos * rover.getModelMatrix().getInverse());
  rover.setRotation(rover.getNumRotations(), delta - roverHeadingAngle, 0, 1, 0);
  cams[1].pan(180 + (delta - roverHeadingAngle));  // rotate in the direction of the rover
  cams[4].pan(delta - roverHeadingAngle);          // rotate in opp direction of the rover

  roverHeadingAngle = delta;
}

//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {
  // --- update cameras
  updateCams();

  if (pathPoints.size() >= 2) {
    moveRover();
  }

  if (mode == PATH_CREATION_MODE || mode == PATH_EDIT_MODE) {
    // -- update the path (curve on surface) made by user
    //
    thePath.clear();
    for_each(pathPoints.begin(), pathPoints.end(), [this](ofVec3f pt) { this->thePath.curveTo(pt); });
  }

  // Update the percantage of path covered
  //
  pct = nextPct();
  // loop back to starting point to restart the animation
  //
  if (pct >= 1) {
    pct = 0;
  }
}

// -- added by sidmishraw
// Gets the next percentage of path completed depending upon the
// current value of the velocity slider.
//
float ofApp::nextPct() { return pct + 0.001f * velSlider; }

// -- added by sidmishraw
// Gets the prev percentage of path completed depending upon the
// current value of the velocity slider.
//
float ofApp::prevPct() { return pct - 0.001f * velSlider; }

// -- added by sidmishraw
// Play the rover movement animation.
//
void ofApp::playAnimation() {
  if (pathPoints.size() > 1) {
    auto strtPt = thePath.getPointAtPercent(0);
    rover.setPosition(strtPt.x, strtPt.y, strtPt.z);
    nextPtIndex++;
  } else {
    log("Not enough points in the path.", 1);
  }
}

// Draws pink colored spheres at the points of the mesh.
//
using namespace std;
void drawPoints(const ofMesh &mesh) {
  auto vertices = mesh.getVertices();
  for_each(vertices.begin(), vertices.end(), [](ofVec3f vertex) {
    ofSetColor(ofColor::red);
    ofDrawSphere(vertex, 0.25);
  });
}

// -- added by sidmishraw ---
// Draws the bounding box around terrain
//
void ofApp::drawBoundingBoxT() {
  ofNoFill();
  ofSetColor(ofColor::white);
  drawBox(boundingBoxT);
}

// -- added by sidmishraw ---
// Draws the bounding box around rover
//
void ofApp::drawBoundingBoxR() {
  if (bRoverLoaded) {
    auto roverMx = rover.getModelMatrix();
    ofPushMatrix();
    ofMultMatrix(roverMx);
    ofNoFill();
    ofSetColor(255, 100, 100);
    drawBox(boundingBoxR);
    ofPopMatrix();

    // -- draw the bounding boxes for rover's components
    //
    for_each(roverCBBoxes.begin(), roverCBBoxes.end(), [this, &roverMx](Box box) {
      ofPushMatrix();
      ofNoFill();
      ofSetColor(0, 244, 33);
      ofMultMatrix(rover.getModelMatrix());
      ofRotate(-90, 1, 0, 0);  // rotate 180 degrees about X axis
      drawBox(box);
      ofPopMatrix();
    });
  }
}

//--------------------------------------------------------------
void ofApp::draw() {
  ofBackground(ofColor::black);

  ofEnableSmoothing();  // moved here by sidmishraw for conflicts with ofxGui
  ofEnableDepthTest();  // moved here by sidmishraw for conflicts with ofxGui

  cams[cameraIndex].begin();

  ofPushMatrix();

  if (bWireframe) {  // wireframe mode  (include axis)

    ofDisableLighting();
    ofSetColor(ofColor::slateGray);
    mars.drawWireframe();

    if (bRoverLoaded) {
      rover.drawWireframe();

      if (bRoverSelected) {
        drawBoundingBoxR();
        drawAxis(rover.getPosition());
      }
    }

    if (bTerrainSelected) {
      drawBoundingBoxT();
      drawAxis(ofVec3f(0, 0, 0));
    }
  } else {
    ofEnableLighting();  // shaded mode
    mars.drawFaces();

    if (bRoverLoaded) {
      rover.drawFaces();

      if (bRoverSelected) {
        drawBoundingBoxR();
        drawAxis(rover.getPosition());
        ofNoFill();
        ofSetColor(ofColor::pink);
      }
    }

    if (bTerrainSelected) {
      drawBoundingBoxT();
      drawAxis(ofVec3f(0, 0, 0));
    }
  }

  if (bDisplayPoints) {  // display points as an option
    glPointSize(3);
    ofSetColor(ofColor::green);
    mars.drawVertices();
  }

  // highlight selected point (draw sphere around selected point)
  //
  if (mode == PATH_CREATION_MODE || mode == PATH_EDIT_MODE) {
    for_each(pathPoints.begin(), pathPoints.end(), [this](ofVec3f p) {
      ofSetColor(255, 0, 0);
      ofNoFill();
      ofDrawSphere(p, 0.11);
    });
  }

  if (mode == PATH_EDIT_MODE) {
    ofSetColor(ofColor::cyan);
    ofNoFill();
    ofDrawSphere(selectedPoint, 0.25);

    if (selectedPtIndex > -1) {
      ofSetColor(255, 255, 0);
      ofNoFill();
      ofDrawSphere(pathPoints[selectedPtIndex], 0.30);
    }
  }

  thePath.draw();

  //  for (int i = 0; i < 5; i++) {
  //    switch (i) {
  //      case 0:
  //        ofSetColor(ofColor::green);
  //        break;
  //      case 1:
  //        ofSetColor(ofColor::blue);
  //        break;
  //      case 2:
  //        ofSetColor(ofColor::red);
  //        break;
  //      case 3:
  //        ofSetColor(ofColor::white);
  //        break;
  //      case 4:
  //        ofSetColor(ofColor::yellow);
  //        break;
  //    }
  //
  //    ofDrawLine(cams[i].getPosition(), cams[i].getTarget().getPosition());
  //  }

  ofPopMatrix();

  cams[cameraIndex].end();

  ofDisableSmoothing();  // moved here by sidmishraw for conflicts with ofxGui
  ofDisableDepthTest();  // moved here by sidmishraw for conflicts with ofxGui

  // -- added by sidmishraw for drawing the slider
  //
  if (tglVelSlider) gui.draw();
}

//

// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {
  ofPushMatrix();

  ofTranslate(location);

  ofSetLineWidth(1.0);

  // X Axis
  ofSetColor(ofColor(255, 0, 0));
  ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));

  // Y Axis
  ofSetColor(ofColor(0, 255, 0));
  ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));

  // Z Axis
  ofSetColor(ofColor(0, 0, 255));
  ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));

  ofPopMatrix();
}

void ofApp::keyPressed(int key) {
  switch (key) {
    case '1': {
      // camera 0
      cameraIndex = 0;
      break;
    }
    case '2': {
      // camera 1
      cameraIndex = 1;
      break;
    }
    case '3': {
      // camera 2
      cameraIndex = 2;
      break;
    }
    case '4': {
      // camera 2
      cameraIndex = 3;
      break;
    }
    case '5': {
      // camera 2
      cameraIndex = 4;
      break;
    }
    case '`': {
      // cycle through cameras
      switchCamera();
      break;
    }

    case OF_KEY_UP: {
      if (bRoverLoaded) {
        roverPos.x += 0.25;
        rover.setPosition(roverPos.x, roverPos.y, roverPos.z);
      }
      break;
    }

    case OF_KEY_DOWN: {
      if (bRoverLoaded) {
        roverPos.x -= 0.25;
        rover.setPosition(roverPos.x, roverPos.y, roverPos.z);
      }
      break;
    }

    case 'p': {
      // play the animation
      bAnimationOn = !bAnimationOn;
      log("Right now animation = " + ofToString(bAnimationOn));
      playAnimation();
      break;
    }

    case 'C':
    case 'c':
      if (cams[cameraIndex].getMouseInputEnabled())
        cams[cameraIndex].disableMouseInput();
      else
        cams[cameraIndex].enableMouseInput();
      break;

    case 'F':
    case 'f':
      ofToggleFullscreen();
      break;

    case 'H':
    case 'h': {
      // toggle GUI slider for rover's velocity
      tglVelSlider = !tglVelSlider;
      break;
    }

    case 'r':
      cams[cameraIndex].reset();
      break;

    case 's':
      savePicture();
      break;

    case 'S': {
      // Toggle point selection mode
      //
      mode = (mode == PATH_CREATION_MODE) ? NORMAL : PATH_CREATION_MODE;
      bPointSelected = !bPointSelected;
      break;
    }

    case 't': {
      // camera re-target to the selected point
      setCameraTarget();
      break;
    }

    case 'T': {
      // select the terrain
      bRoverSelected = false;
      bTerrainSelected = true;
      break;
    }

    case 'R': {
      // select the rover and deselect the terrain
      bRoverSelected = true;
      bTerrainSelected = false;
      break;
    }

    case 'u': {
      // Toggle path edit mode
      //
      mode = (mode == PATH_EDIT_MODE) ? NORMAL : PATH_EDIT_MODE;
      selectedPtIndex = -1;  // reset for this session
      break;
    }

    case 'v':
      togglePointsDisplay();
      break;

    case 'V':
      break;

    case 'w':
      toggleWireframeMode();
      break;

    case OF_KEY_ALT:
      cams[cameraIndex].enableMouseInput();
      bAltKeyDown = true;
      break;

    case OF_KEY_CONTROL:
      bCtrlKeyDown = true;
      break;

    case OF_KEY_SHIFT:
      break;

    case OF_KEY_DEL:
      break;

    default:
      break;
  }
}

void ofApp::toggleWireframeMode() { bWireframe = !bWireframe; }

void ofApp::toggleSelectTerrain() { bTerrainSelected = !bTerrainSelected; }

void ofApp::togglePointsDisplay() { bDisplayPoints = !bDisplayPoints; }

void ofApp::keyReleased(int key) {
  switch (key) {
    case OF_KEY_ALT:
      cams[cameraIndex].disableMouseInput();
      bAltKeyDown = false;
      break;
    case OF_KEY_CONTROL:
      bCtrlKeyDown = false;
      break;
    case OF_KEY_SHIFT:
      break;
    default:
      break;
  }
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {}

// -- added by sidmishraw --
// Checks if the rover was selected with the mouse click by
// checking if the ray intersects any of the bounding boxes of the
// rover's components or the rover itself.
//
bool ofApp::roverSelected(const ofVec3f &mousePoint) {
  bool hit = false;

  ofVec3f rayPoint = cams[cameraIndex].screenToWorld(mousePoint) * rover.getModelMatrix().getInverse();
  ofVec3f rayDir = rayPoint - (cams[cameraIndex].getPosition() * rover.getModelMatrix().getInverse());
  rayDir.normalize();

  Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z), Vector3(rayDir.x, rayDir.y, rayDir.z));

  for_each(roverCBBoxes.begin(), roverCBBoxes.end(), [this, &hit, &ray](Box b) {
    if (b.intersect(ray, -100, 100)) {
      hit = true;
    }
  });

  if (!hit) {
    hit = boundingBoxR.intersect(ray, -100, 100);
  }

  return hit;
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
  bMouseDown = true;
  mousePoint = ofVec3f(mouseX, mouseY);  // added by sidmishraw for drawing ray

  ofVec3f mouse(mouseX, mouseY);
  ofVec3f rayPoint = cams[cameraIndex].screenToWorld(mouse);
  ofVec3f rayDir = rayPoint - cams[cameraIndex].getPosition();
  rayDir.normalize();
  Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z), Vector3(rayDir.x, rayDir.y, rayDir.z));

  if (bRoverLoaded && roverSelected(mouse)) {
    bRoverSelected = true;
    bTerrainSelected = false;

    auto rmx = boundingBoxR.max();
    auto rmi = boundingBoxR.min();
    auto c = (rmx - rmi) / 2.0;
    selectedPoint = ofVec3f(c.x(), c.y(), c.z()) * rover.getModelMatrix();
    log("Center - rover - bb = " + ofToString(selectedPoint));

  } else if (boundingBoxT.intersect(ray, -100, 100)) {
    bTerrainSelected = true;
    bRoverSelected = false;

    // --- Points selection for making a path
    //
    if (mode == PATH_CREATION_MODE) {
      // Adding points to path
      //
      auto maybePt = octtreeT->search(ray, -100, 100);  // fetch the point from octtree
      if (maybePt->isPresent()) {
        auto pt = maybePt->get();
        pathPoints.push_back(pt);
      }
    }

    // -- Points selection to edit the path
    //
    if (mode == PATH_EDIT_MODE) {
      // Editing a point on the path
      //
      auto maybePt = octtreeT->search(ray, -100, 100);  // fetch the point from octtree
      if (maybePt->isPresent()) {
        auto pt = maybePt->get();
        auto loc = find(pathPoints.begin(), pathPoints.end(), pt);

        if (loc != pathPoints.end()) {
          selectedPoint = pt;
          selectedPtIndex = loc - pathPoints.begin();
          log("Selected pt = " + ofToString(selectedPoint));
          log("Selected pt index = " + ofToString(selectedPtIndex));
        }
      }
    }
  } else {
    bTerrainSelected = false;
    bRoverSelected = false;
  }
}

// Draw a box from a "Box" class
//
void ofApp::drawBox(const Box &box) {
  // max, min bounds using the Box class for ray intersection testing.
  //
  Vector3 min = box.parameters[0];
  Vector3 max = box.parameters[1];

  Vector3 size = max - min;
  Vector3 center = size / 2 + min;

  ofVec3f p = ofVec3f(center.x(), center.y(), center.z());

  float w = size.x();
  float h = size.y();
  float d = size.z();

  ofDrawBox(p, w, h, d);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box ofApp::meshBounds(const ofMesh &mesh) {
  int n = mesh.getNumVertices();

  ofVec3f v = mesh.getVertex(0);

  ofVec3f max = v;
  ofVec3f min = v;

  for (int i = 1; i < n; i++) {
    ofVec3f v = mesh.getVertex(i);

    if (v.x > max.x)
      max.x = v.x;
    else if (v.x < min.x)
      min.x = v.x;

    if (v.y > max.y)
      max.y = v.y;
    else if (v.y < min.y)
      min.y = v.y;

    if (v.z > max.z)
      max.z = v.z;
    else if (v.z < min.z)
      min.z = v.z;
  }

  auto minB = Vector3(min.x, min.y, min.z);
  auto maxB = Vector3(max.x, max.y, max.z);

  return Box(minB, maxB);
}

//  Subdivide a Box into eight(8) equal size boxes, return them in boxList;
//
void ofApp::subDivideBox8(const Box &box, vector<Box> &boxList) {
  Vector3 min = box.parameters[0];
  Vector3 max = box.parameters[1];

  Vector3 size = max - min;
  Vector3 center = size / 2 + min;

  float xdist = (max.x() - min.x()) / 2;
  float ydist = (max.y() - min.y()) / 2;
  float zdist = (max.z() - min.z()) / 2;

  Vector3 h = Vector3(0, ydist, 0);

  //  generate ground floor
  //
  Box b[8];
  b[0] = Box(min, center);
  b[1] = Box(b[0].min() + Vector3(xdist, 0, 0), b[0].max() + Vector3(xdist, 0, 0));
  b[2] = Box(b[1].min() + Vector3(0, 0, zdist), b[1].max() + Vector3(0, 0, zdist));
  b[3] = Box(b[2].min() + Vector3(-xdist, 0, 0), b[2].max() + Vector3(-xdist, 0, 0));

  boxList.clear();
  for (int i = 0; i < 4; i++) boxList.push_back(b[i]);

  // generate second story
  //
  for (int i = 4; i < 8; i++) {
    b[i] = Box(b[i - 4].min() + h, b[i - 4].max() + h);
    boxList.push_back(b[i]);
  }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
  mousePoint = ofVec3f(mouseX, mouseY);  // added by sidmishraw for drawing ray
  ofVec3f mouse(mouseX, mouseY);
  ofVec3f rayPoint = cams[cameraIndex].screenToWorld(mouse);
  ofVec3f rayDir = rayPoint - cams[cameraIndex].getPosition();
  rayDir.normalize();
  Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z), Vector3(rayDir.x, rayDir.y, rayDir.z));

  // -- Points selection to edit the path
  //
  if (mode == PATH_EDIT_MODE) {
    // Editing a point on the path
    //
    auto maybePt = octtreeT->search(ray, -100, 100);  // fetch the point from octtree
    if (maybePt->isPresent()) {
      auto pt = maybePt->get();
      selectedPoint = pt;
    }
  }

  bMouseDown = false;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
  bMouseDown = false;
  if (mode == PATH_EDIT_MODE) {
    if (selectedPtIndex > -1) {
      pathPoints[selectedPtIndex] = selectedPoint;  // replace with new point
    }
  }
}

//
//  Select Target Point on Terrain by comparing distance of mouse to
//  vertice points projected onto screenspace.
//  if a point is selected, return true, else return false;
//
bool ofApp::doPointSelection() {
  ofMesh mesh = mars.getMesh(0);
  int n = mesh.getNumVertices();
  float nearestDistance = 0;
  int nearestIndex = 0;

  bPointSelected = false;

  ofVec2f mouse(mouseX, mouseY);
  vector<ofVec3f> selection;

  // We check through the mesh vertices to see which ones
  // are "close" to the mouse point in screen space.  If we find
  // points that are close, we store them in a vector (dynamic array)
  //
  for (int i = 0; i < n; i++) {
    ofVec3f vert = mesh.getVertex(i);
    ofVec3f posScreen = cams[cameraIndex].worldToScreen(vert);
    float distance = posScreen.distance(mouse);
    if (distance < selectionRange) {
      selection.push_back(vert);
      bPointSelected = true;
    }
  }

  //  if we found selected points, we need to determine which
  //  one is closest to the eye (camera). That one is our selected target.
  //
  if (bPointSelected) {
    float distance = 0;
    for (int i = 0; i < selection.size(); i++) {
      ofVec3f point = cams[cameraIndex].worldToCamera(selection[i]);

      // In camera space, the camera is at (0,0,0), so distance from
      // the camera is simply the length of the point vector
      //
      float curDist = point.length();

      if (i == 0 || curDist < distance) {
        distance = curDist;
        selectedPoint = selection[i];
      }
    }
  }
  return bPointSelected;
}

// Set the camera to use the selected point as it's new target
//
void ofApp::setCameraTarget() { cams[0].setTarget(selectedPoint); }

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {}

//--------------------------------------------------------------
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//
void ofApp::initLightingAndMaterials() {
  static float ambient[] = {.5f, .5f, .5, 1.0f};
  static float diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};

  static float position[] = {5.0, 5.0, 5.0, 0.0};

  static float lmodel_ambient[] = {1.0f, 1.0f, 1.0f, 1.0f};

  static float lmodel_twoside[] = {GL_TRUE};

  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, position);

  glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, position);

  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
  glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  //	glEnable(GL_LIGHT1);
  glShadeModel(GL_SMOOTH);
}

void ofApp::savePicture() {
  ofImage picture;

  picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
  picture.save("screenshot.png");

  log("picture saved", 1);
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent(ofDragInfo dragInfo) {
  ofVec3f point;
  mouseIntersectPlane(ofVec3f(0, 0, 0), cams[cameraIndex].getZAxis(), point);

  if (rover.loadModel(dragInfo.files[0])) {
    rover.setScaleNormalization(false);
    rover.setScale(.25, .25, .25);
    rover.setPosition(point.x, point.y, point.z);

    log("Rover position = " + ofToString(rover.getPosition()));

    bRoverLoaded = true;
    bRoverSelected = true;

    auto pMin = rover.getSceneMin();
    auto pMax = rover.getSceneMax();

    // the rover's bounding box (largest)
    boundingBoxR = Box(Vector3(pMin.x, pMin.y, pMin.z), Vector3(pMax.x, pMax.y, pMax.z));
    // the rover's component's bounding boxes (smaller ones)
    for (int i = 0; i < rover.getMeshCount(); i++) {
      roverCBBoxes.push_back(meshBounds(rover.getMesh(i)));
    }

  } else {
    log("Error: Can't load model" + ofToString(dragInfo.files[0]), 2);
  }
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
  ofVec2f mouse(mouseX, mouseY);
  ofVec3f rayPoint = cams[cameraIndex].screenToWorld(mouse);
  ofVec3f rayDir = rayPoint - cams[cameraIndex].getPosition();
  rayDir.normalize();
  return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}

// -- added by sidmishraw
// Switch the camera to view from.
//
void ofApp::switchCamera() {
  cameraIndex = (cameraIndex + 1) % 5;  // to keep the index
  log("Viewing through camera #" + ofToString(cameraIndex));
}

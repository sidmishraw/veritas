//
//  octtree.cpp
//  martian-terrain
//
//  Created by Sidharth Mishra on 4/17/18.
//

#include "octtree.h"

using namespace sidmishraw_octtree;
using namespace std;

// Converts from a Vector3 instance to ofVec3f.
//
ofVec3f convert(const Vector3 &p) { return ofVec3f(p.x(), p.y(), p.z()); }

// Checks if a point p is within the bounds defined by min and max.
//
bool isWithinBounds(Vector3 min, Vector3 max, ofVec3f p) {
  if ((p.x >= min.x() && p.x <= max.x()) && (p.y >= min.y() && p.y <= max.y()) && (p.z >= min.z() && p.z <= max.z()))
    return true;

  return false;
}

// Computes the mesh bounds.
//
vector<Vector3> meshBounds(const ofMesh &mesh) {
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

  vector<Vector3> bounds;
  bounds.push_back(Vector3(min.x, min.y, min.z));
  bounds.push_back(Vector3(max.x, max.y, max.z));

  return bounds;
}

//--------------- OCTTREENODE - STARTS -----------------------------------------
//
//
OctTreeNode::OctTreeNode(Vector3 min, Vector3 max, int d, weak_ptr<OctTree> tree, vector<int> indices) {
  box = Box(min, max);
  isLeaf = false;
  depth = d;
  shouldLightUp = false;

  // Get the shared_ptr instance of octtree if it still exists.
  //
  octTree = tree.lock();

  // Check in the points within this node's bounds into its
  // pointIndices vector.
  //
  if (octTree) {
    for_each(indices.begin(), indices.end(), [this, &min, &max](int index) {
      auto vertex = octTree->mesh.getVertex(index);
      if (isWithinBounds(min, max, vertex)) this->pointIndices.push_back(index);
    });
  }
}

void OctTreeNode::subdivide() {
  // if (depth > octTree->MAX_DEPTH) return; // bail out after reaching max
  // depth

  // no more points left to make children
  //
  if (this->pointIndices.size() <= 1) {
    isLeaf = true;
    return;
  }

  // cout << "subdividing --- " << depth << " size = " <<
  // this->pointIndices.size() << endl;

  Vector3 min = box.parameters[0];
  Vector3 max = box.parameters[1];

  Vector3 size = max - min;
  Vector3 center = size / 2 + min;

  float xDist = (max.x() - min.x()) / 2;
  float yDist = (max.y() - min.y()) / 2;
  float zDist = (max.z() - min.z()) / 2;

  Vector3 h = Vector3(0, yDist, 0);

  // Generate children
  //
  shared_ptr<OctTreeNode> n[8];
  n[0] = make_shared<OctTreeNode>(min, center, depth + 1, octTree, this->pointIndices);
  n[1] = make_shared<OctTreeNode>(n[0]->box.min() + Vector3(xDist, 0, 0), n[0]->box.max() + Vector3(xDist, 0, 0),
                                  depth + 1, octTree, this->pointIndices);
  n[2] = make_shared<OctTreeNode>(n[1]->box.min() + Vector3(0, 0, zDist), n[1]->box.max() + Vector3(0, 0, zDist),
                                  depth + 1, octTree, this->pointIndices);
  n[3] = make_shared<OctTreeNode>(n[2]->box.min() + Vector3(-xDist, 0, 0), n[2]->box.max() + Vector3(-xDist, 0, 0),
                                  depth + 1, octTree, this->pointIndices);
  for (int i = 4; i < 8; i++) {
    n[i] = make_shared<OctTreeNode>(n[i - 4]->box.min() + h, n[i - 4]->box.max() + h, depth + 1, octTree,
                                    this->pointIndices);
  }

  for (int i = 0; i < 8; i++) children.push_back(n[i]);

  // cout << "my children = " << children.size() << endl;

  for_each(children.begin(), children.end(), [](shared_ptr<OctTreeNode> child) { child->subdivide(); });
}

// render the octree node as a box
//
void OctTreeNode::render() {
  if (depth > octTree->MAX_DEPTH) return;  // bail out after reaching max depth.
  if (pointIndices.size() < 1) return;     // don't render empty nodes

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

  if (shouldLightUp)
    ofSetColor(ofColor::red);
  else
    ofSetColor(ofColor::white);

  ofDrawBox(p, w, h, d);

  // Render children recursively.
  //
  for_each(children.begin(), children.end(), [](shared_ptr<OctTreeNode> child) { child->render(); });
}

// Check if the ray intersects a node. If it intersects,
// do deeper in the hierarchy till we reach the leaf node with just 1 vertex.
// Then, we return the vertex in the leaf node.
// Else, we return an empty maybepoint object.
//
void OctTreeNode::intersects(const Ray &r, float t0, float t1) {
  shouldLightUp = box.intersect(r, t0, t1);

  // Sparse box, don't light up
  //
  if (pointIndices.size() < 1)
    shouldLightUp = false;
  else if (shouldLightUp) {
    if (isLeaf) {
      octTree->thePoint.set(octTree->mesh.getVertex(pointIndices[0]));

      //      log("-----");
      //      log("Nbr of indices = " + ofToString(pointIndices.size()));
    } else {
      for_each(children.begin(), children.end(),
               [&r, t0, t1](shared_ptr<OctTreeNode> child) { child->intersects(r, t0, t1); });
    }
  }
}

void OctTreeNode::clearSelection() {
  shouldLightUp = false;
  for_each(children.begin(), children.end(), [](shared_ptr<OctTreeNode> child) { child->clearSelection(); });
}

//
//
//--------------- OCTTREENODE - ENDS -------------------------------------------

// ---------------- OCTTREE - STARTS -------------------------------------------
//
//

void OctTree::generate(const ofMesh &mesh, int maxLevel) {
  this->mesh = mesh;
  MAX_DEPTH = maxLevel;

  vector<int> indices;
  for (int i = 0; i < mesh.getVertices().size(); i++) indices.push_back(i);

  vector<Vector3> bounds = meshBounds(mesh);
  root = make_shared<OctTreeNode>(bounds[0], bounds[1], 0, shared_from_this(), indices);
  root->subdivide();
}

void OctTree::render() {
  root->render();

  ofSetColor(ofColor::cyan);
  ofDrawSphere(thePoint.get(), 0.25);
}

shared_ptr<MaybePoint> OctTree::search(const Ray &r, float t0, float t1) {
  root->clearSelection();
  thePoint.clear();

  root->intersects(r, t0, t1);

  auto p = make_shared<MaybePoint>();

  if (thePoint.isPresent()) {
    p->set(thePoint.get());
  }

  return p;
}

//
//
// ---------------- OCTTREE - ENDS-- -------------------------------------------

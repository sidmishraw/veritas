//
//  octtree.h
//  martian-terrain
//
//  Created by Sidharth Mishra on 4/16/18.
//

#ifndef octtree_h
#define octtree_h

#include <memory>
#include <vector>

#include "ofMain.h"

#include "Util.h"
#include "box.h"
#include "ray.h"

// The sidmishraw_octtree namespace contains all the OctTree related
// utilities.
//
namespace sidmishraw_octtree {

// The maybe type for optional values
//
class MaybePoint {
  bool bPresent;
  ofVec3f point;

 public:
  // Create maybe without a point
  //
  MaybePoint() { bPresent = false; }

  // Create Maybe with a point.
  //
  MaybePoint(ofVec3f p) {
    bPresent = true;
    point = point;
  }

  // Checks if there is a point. True means a point was found,
  // else false.
  //
  bool isPresent() { return this->bPresent; }

  // Clears the contents of this maybe container, signifying that
  // the current value is no longer valid.
  //
  void clear() { this->bPresent = false; }

  // Returns the point that is present.
  //
  ofVec3f get() { return this->point; }

  // Places a point in the maybe
  //
  void set(ofVec3f p) {
    bPresent = true;
    point = p;
  }
};

class OctTree;  // forward declaring OctTree class

//---------------------------------------------------------------
// OctTreeNode is a node in the OctTree data structure.
//
using namespace std;
class OctTreeNode : public enable_shared_from_this<OctTreeNode> {
 public:
  // Creates a new OctTreeNode for the given:
  // min bound,
  // max bound,
  // depth at which this OctTreeNode is present at,
  // the octtree that this node belongs to, and
  // the indices of the vertices belonging to it.
  //
  OctTreeNode(Vector3 min, Vector3 max, int d, weak_ptr<OctTree> tree, vector<int> indices);

  // ----------- ATTRIBUTES -----------------

  // The octtree to which this node belongs
  //
  shared_ptr<OctTree> octTree;

  // The box that represents the octtree node.
  //
  Box box;

  // Indices of the points that belong to this node.
  //
  vector<int> pointIndices;

  // The children of this node. The leaf node has no children.
  // Hence, all the children will be NULL pointers.
  // To save myself this trouble, I'll add in a boolean flag
  // to denote if this node is a leaf node.
  //
  vector<shared_ptr<OctTreeNode>> children;

  // This flag indicates if the node is a leaf or not.
  //
  bool isLeaf;

  // Depth this node belongs to.
  //
  int depth;

  // This flag indicates if the node should light up.
  // The node lights up when a ray intersects it.
  //
  bool shouldLightUp;

  // ----------- OPERATIONS ------------------

  // Draws the OctTreeNode.
  //
  void render();

  // Subdivides this node to generate children nodes.
  //
  void subdivide();

  // Checks if the ray r intersects this node.
  //
  void intersects(const Ray& r, float t0, float t1);

  // Clears the shouldLightUp bit.
  //
  void clearSelection();
};

//---------------------------------------------------------------
// OctTree is a data structure for fast ray intersection testing.
//
using namespace std;
class OctTree : public enable_shared_from_this<OctTree> {
 public:
  // ----------- ATTRIBUTES -----------------

  // Max depth to render.
  //
  int MAX_DEPTH;

  // The point selected and to render as a sphere's center.
  //
  MaybePoint thePoint;

  // The mesh for which this octtree is being generated.
  //
  ofMesh mesh;

  // The root node of the octtree.
  //
  shared_ptr<OctTreeNode> root;

  // ----------- OPERATIONS ------------------

  // Generates this OctTree from the given mesh.
  //
  void generate(const ofMesh& mesh, int maxLevel);

  // Renders this OctTree.
  //
  void render();

  // Searches the point of intersection given the ray.
  //
  shared_ptr<MaybePoint> search(const Ray& r, float t0, float t1);
};

};  // namespace sidmishraw_octtree

#endif /* octtree_h */

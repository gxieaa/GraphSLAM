#include <iostream>
#include <Eigen/Core>
#include <limits.h>
#include <algorithm>

#include "g2o/core/sparse_optimizer.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

bool incDataAssociation (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance);

bool fullDataAssociation (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance);

bool correspondenceTest (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double xi);

bool distantTest (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double maxDistance);

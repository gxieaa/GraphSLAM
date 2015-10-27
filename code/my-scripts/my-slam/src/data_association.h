#include <iostream>
#include <Eigen/Core>
#include <limits.h>
#include <algorithm>

#include "g2o/core/sparse_optimizer.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

bool dataAssociation (SparseOptimizer& optimizer, double xi, double maxDistance);

bool dataAssociation2 (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance);

bool dataAssociation3 (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance);

int getMinInd (set<HyperGraph::Edge*> &es, OptimizableGraph::VertexContainer &vc);

double getMaxVar (SparseOptimizer& optimizer, OptimizableGraph::VertexContainer &vc);

double getMaxVarLandmark (SparseOptimizer &optimizer, OptimizableGraph::Vertex* v1);

bool correspondenceTest (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double xi);

bool sharePose (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);

bool distantTest (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double varDistance);

void makeAssociation (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);

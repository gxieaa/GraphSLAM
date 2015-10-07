#include <iostream>
#include <Eigen/Core>

#include "g2o/core/sparse_optimizer.h"

#include "util_functs.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

bool data_association (SparseOptimizer& optimizer, double xi);

double get_max_var (SparseOptimizer& optimizer, OptimizableGraph::VertexContainer &vc);

bool correspondence_test (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double xi);

bool share_pose (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);

HyperGraph::Vertex* extract_other_vertex (HyperGraph::Edge* edge, OptimizableGraph::Vertex* vertex);

bool distant_test (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double varDistance);

void make_association (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);

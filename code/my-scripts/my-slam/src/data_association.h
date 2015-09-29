#include <iostream>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/core/factory.h"

#include "g2o/stuff/command_args.h"

#include <Eigen/Core>

#include <typeinfo>

using namespace std;
using namespace g2o;
using namespace Eigen;

bool data_association (SparseOptimizer& optimizer, double xi);

bool correspondence_test (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double xi);

bool share_pose (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);

HyperGraph::Vertex* extract_other_vertex (HyperGraph::Edge* edge, OptimizableGraph::Vertex* vertex);

bool distant_test (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);

void make_association (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);

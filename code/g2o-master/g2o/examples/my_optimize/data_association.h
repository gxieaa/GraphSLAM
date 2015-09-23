#include <iostream>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/core/factory.h"
//#include "g2o/types/slam3d/types_slam3d.h"
//#include "g2o/types/slam2d/types_slam2d.h"
//#include "g2o/types/slam2d/vertex_se2.h"

#include "g2o/stuff/command_args.h"

#include <Eigen/Core>

#include <typeinfo>

using namespace std;
using namespace g2o;
using namespace Eigen;

void data_association (SparseOptimizer& optimizer);

bool correspondence_test (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);

void make_association(OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);
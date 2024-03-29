#include <iostream>
#include <string>
#include <fstream>
#include <Eigen/Core>

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/sparse_optimizer.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

void listRobustKernels (bool list);

void readDataFile (ifstream &ifs, string filename);

void writeDataFile (string outputFilename, SparseOptimizer &optimizer);

void loadRobustKernel (string robustKernel, bool nonSequential, double huberWidth, SparseOptimizer &optimizer);

void getAllPoses (SparseOptimizer &optimizer, OptimizableGraph::VertexContainer &poses);

double getMaxDistance (double xi);

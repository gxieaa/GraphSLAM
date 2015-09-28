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

#include <typeinfo>

#include "data_association.h"

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

int main(int argc, char** argv)
{
  // Command line parsing
  int maxIterations;
  string outputFilename;
  string inputFilename;
  double xi;
  CommandArgs arg;
  arg.param("i", maxIterations, 10, "perform n iterations, if negative consider the gain");
  arg.param("o", outputFilename, "", "output final version of the graph");
  arg.param("t", xi, 10.0, "threshold for data association");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed");
  arg.parseArgs(argc, argv);

  // create the linear solver
  BlockSolverX::LinearSolverType * linearSolver = new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();

  // create the block solver on top of the linear solver
  BlockSolverX* blockSolver = new BlockSolverX(linearSolver);

  // create the algorithm to carry out the optimization
  //OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
  OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);

  // NOTE: We skip to fix a variable here, either this is stored in the file
  // itself or Levenberg will handle it.

  // create the optimizer to load the data and carry out the optimization
  SparseOptimizer optimizer;
  optimizer.setVerbose(true);
  optimizer.setAlgorithm(optimizationAlgorithm);

  ifstream ifs;
  ifs.open(inputFilename.c_str());
  if (! ifs) {
    cerr << "unable to open " << inputFilename << endl;
    return 1;
  }
  
  optimizer.load(ifs); 
  ifs.close();
  optimizer.initializeOptimization();
  optimizer.optimize(maxIterations);
  
  while(true) {
      // data association
      bool no_more_association  = data_association(optimizer, xi);
      
      // output file
      if (outputFilename.size() > 0) {
        if (outputFilename == "-") {
          cerr << "saving to stdout";
          optimizer.save(cout);
        } else {
          cerr << "saving " << outputFilename << " ... ";
          optimizer.save(outputFilename.c_str());
        }
        cerr << "done." << endl;
      }
      
      // finished test
      if (no_more_association) break;
      
      // open data file
      ifs.open(outputFilename.c_str());
      if (! ifs) {
        cerr << "unable to open " << outputFilename << endl;
        return 1;
      }
      
      // optimize
      optimizer.clear();
      optimizer.load(ifs); 
      ifs.close();
      optimizer.initializeOptimization();
      optimizer.optimize(maxIterations);
  }
  
  return 0;
}

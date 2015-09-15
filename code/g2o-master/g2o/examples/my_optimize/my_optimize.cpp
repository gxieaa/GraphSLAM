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
#include <Eigen/Eigenvalues>

#include <typeinfo>

using namespace std;
using namespace g2o;
using namespace Eigen;


// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

int main(int argc, char** argv)
{
  // Command line parsing
  int maxIterations;
  string outputFilename;
  string inputFilename;
  CommandArgs arg;
  arg.param("i", maxIterations, 10, "perform n iterations, if negative consider the gain");
  arg.param("o", outputFilename, "", "output final version of the graph");
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

  ifstream ifs(inputFilename.c_str());
  if (! ifs) {
    cerr << "unable to open " << inputFilename << endl;
    return 1;
  }
  optimizer.load(ifs);
  optimizer.initializeOptimization();
  optimizer.optimize(maxIterations);
  
  // data association
  OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
  // iterate through landmarks
  for (size_t i=0; i<vc.size(); ++i) {
    OptimizableGraph::Vertex* v1 = vc[i];
    if (v1->dimension() == 2) { // check if vertex is landmark
      for (size_t j=i+1; j<vc.size(); ++j){
        OptimizableGraph::Vertex* v2 = vc[j];
        if (v2->dimension() == 2) { // check if vertex is landmark
          std::vector<std::pair<int, int> > blockIndices;
          blockIndices.push_back(make_pair(v1->hessianIndex(), v1->hessianIndex()));
          blockIndices.push_back(make_pair(v1->hessianIndex(), v2->hessianIndex()));
          blockIndices.push_back(make_pair(v2->hessianIndex(), v1->hessianIndex()));
          blockIndices.push_back(make_pair(v2->hessianIndex(), v2->hessianIndex()));
          SparseBlockMatrix<MatrixXd> spinv;
          optimizer.computeMarginals(spinv, blockIndices);
          // get marginal covariance and information matrix between pair of landmarks
          Matrix4d marginCovMat;
          marginCovMat.setZero();
          marginCovMat.topLeftCorner(2, 2) = *(spinv.block(v1->hessianIndex(), v1->hessianIndex()));
          marginCovMat.topRightCorner(2, 2) = *(spinv.block(v1->hessianIndex(), v2->hessianIndex()));
          marginCovMat.bottomLeftCorner(2, 2) = *(spinv.block(v2->hessianIndex(), v1->hessianIndex()));
          marginCovMat.bottomRightCorner(2, 2) = *(spinv.block(v2->hessianIndex(), v2->hessianIndex()));
          //cout << "Marginal Covariance Matrix (v" << v1->id() << ", " << "v" << v2->id() << "):" << endl;
          //cout << marginCovMat << endl;
          Matrix4d marginInfoMat = marginCovMat.inverse();
          cout << "Marginal Information Matrix (v" << v1->id() << ", " << "v" << v2->id() << "):" << endl;
          cout << marginInfoMat << endl;
          // get landmark estimate vectors
          std::vector<double> v1EstVec;
          v1->getEstimateData(v1EstVec);
          Vector2d v1Est(v1EstVec.data());
          std::vector<double> v2EstVec;
          v2->getEstimateData(v2EstVec);
          Vector2d v2Est(v2EstVec.data());
          Vector4d vEst;
          vEst << v1Est, v2Est;
          cout << "estData (v" << v1->id() << ", " << "v" << v2->id() << "):" << endl;
          cout << vEst << endl;
          // get information vector
          Vector4d infoVec = marginInfoMat * vEst;
          cout << "infoVec (v" << v1->id() << ", " << "v" << v2->id() << "):" << endl;
          cout << infoVec << endl;
          // get landmark difference information matrix
          Matrix<double, 4, 2> diffMat;
          diffMat << Matrix2d::Identity(2,2), -1*Matrix2d::Identity(2,2);
          Matrix2d diffInfoMat = diffMat.transpose() * marginInfoMat * diffMat;
          cout << "diffInfoMat (v" << v1->id() << ", " << "v" << v2->id() << "):" << endl;
          cout << diffInfoMat << endl;
          // get landmark difference information vector
          Vector2d diffInfoVec = diffMat.transpose() * infoVec;
          cout << "diffInfovec (v" << v1->id() << ", " << "v" << v2->id() << "):" << endl;
          cout << diffInfoVec << endl;
          // get landmark difference estimation vector
          Vector2d diffEstVec = diffInfoMat.inverse() * diffInfoVec;
          cout << "diffEstVec (v" << v1->id() << ", " << "v" << v2->id() << "):" << endl;
          cout << diffEstVec << endl;
          // get likelihood
          double likelihood;
          likelihood = pow((2*M_PI * diffInfoMat.inverse()).determinant(), -0.5) * exp(-0.5 * diffEstVec.transpose() * diffInfoMat.inverse() * diffEstVec);
          cout << "likelihood (v" << v1->id() << ", " << "v" << v2->id() << "):" << endl;
          cout << likelihood << endl;
        }
      }
    }
  }
  
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
  return 0;
}

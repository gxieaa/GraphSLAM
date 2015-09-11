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
  
  // print uncertainty
  OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
  
  //for (OptimizableGraph::VertexContainer::const_iterator it=vc.begin(); it!=vc.end(); ++it) {
  //  OptimizableGraph::Vertex* v= *it;
  //  if (v->dimension() == 2){
  //    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::ColMajor>>((*v).hessianData()) << '\n'<<'\n';
  //  }
  //  else if (v->dimension() == 3){
  //    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::ColMajor>>((*v).hessianData()) << '\n'<<'\n';
  //  }
  //}
  
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
          // correspondence test
          Matrix4d marginMat;
          marginMat.setZero();
          marginMat.topLeftCorner(2, 2) = *(spinv.block(v1->hessianIndex(), v1->hessianIndex()));
          marginMat.topRightCorner(2, 2) = *(spinv.block(v1->hessianIndex(), v2->hessianIndex()));
          marginMat.bottomLeftCorner(2, 2) = *(spinv.block(v2->hessianIndex(), v1->hessianIndex()));
          marginMat.bottomRightCorner(2, 2) = *(spinv.block(v2->hessianIndex(), v2->hessianIndex()));
          cout << "Marginal Covariance Matrix (v" << v1->id() << ", " << "v" << v2->id() << "):" << endl;
          cout << marginMat << endl;
          EigenSolver<MatrixXd> es(marginMat);
          cout << "Eigenvalues:" << endl << es.eigenvalues() << endl;
          cout << "Eigenvectors:" << endl << es.eigenvectors() << endl;
          //std::cout << spinv << std::endl;
          //std::cout << *(spinv.block(v1->hessianIndex(), v1->hessianIndex())) << ", " << *(spinv.block(v1->hessianIndex(), v2->hessianIndex())) << ", " << *(spinv.block(v2->hessianIndex(), v1->hessianIndex())) << ", " << *(spinv.block(v2->hessianIndex(), v2->hessianIndex())) << "." << std::endl;
          
        }
      }
    }
  }
  
  //OptimizableGraph::VertexContainer pairVertex;
  //pairVertex.push_back(vc[1]);
  //pairVertex.push_back(vc[6]);
  //SparseBlockMatrix<MatrixXd> spinv;
  //optimizer.computeMarginals(spinv, pairVertex);
  //std::cout << spinv << std::endl;
  //for (size_t i=0; i<vc.size()-5; ++i) {
  //  for (size_t j=0; j<vc.size()-5; ++j) {
  //    cout << "Block " << i << ", " << j << ":" << endl;
  //    cout << *(spinv.block(vc[i]->hessianIndex(), vc[j]->hessianIndex())) << endl;
  //  }
  //}
  //cout << "Block " << 5 << ", " << 5 << ":" << endl;
  //    cout << *(spinv.block(vc[5]->hessianIndex(), vc[5]->hessianIndex())) << endl;
  
  
 //for (size_t i=0; i<vc.size(); ++i) {
 //   OptimizableGraph::Vertex* v1 = vc[i];
 //   OptimizableGraph::VertexContainer allVertex;
 //   allVertex.push_back(v1);
 //   SparseBlockMatrix<MatrixXd> spinv;
 //   optimizer.computeMarginals(spinv, allVertex);
 //   std::cout << spinv << std::endl;
 //} 
  
  //SparseBlockMatrix<MatrixXd> spinv;
  //optimizer.computeMarginals(spinv, vc);
  //std::cout << spinv << std::endl;
  
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

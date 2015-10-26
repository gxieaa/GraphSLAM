#include <iostream>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/command_args.h"

#include <typeinfo>
#include <ctime>

#include <gperftools/profiler.h>

#include "data_association.h"
#include "slam_functs.h"

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

int main(int argc, char** argv) {

    // clock for time measurement
    ProfilerStart("my_slam");
    clock_t begin = clock();

    // Command line parsing
    int iteration = 0;
    int maxIterations;
    double xi;
    string robustKernel;
    double huberWidth;
    bool listKernelsBool;
    bool nonSequential;
    string outputFilename;
    string inputFilename;
    CommandArgs arg;

    arg.param("i", maxIterations, 10, "perform n iterations, if negative consider the gain");
    arg.param("t", xi, 1.0, "threshold for data association");
    arg.param("robustKernel", robustKernel, "", "use this robust error function");
    arg.param("robustKernelWidth", huberWidth, -1., "width for the robust Kernel (only if robustKernel)");
    arg.param("listRobustKernels", listKernelsBool, false, "list the registered robust kernels");
    arg.param("nonSequential", nonSequential, false, "apply the robust kernel only on loop closures and not odometries");
    arg.param("o", outputFilename, "", "output final version of the graph");
    arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed");
    arg.parseArgs(argc, argv);

    // list robust kernel
    listRobustKernels (listKernelsBool);

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

    // read data file and load to optimizer
    ifstream ifs;
    readDataFile (ifs, inputFilename);
    optimizer.load(ifs);
    ifs.close();

    // load robust kernel
    loadRobustKernel (robustKernel, nonSequential,  huberWidth, optimizer);

    // initial guess
    optimizer.initializeOptimization();
    optimizer.optimize(maxIterations);
    
    // get pose vertices
    OptimizableGraph::VertexContainer poses;
    getAllPoses (optimizer, poses);
   
    // compute max distance
    double maxDistance = getMaxDistance (optimizer, xi);
     
    // poses loop
    //cout << "poses: " << poses.size() << endl; 
    for (int i=0; i<poses.size(); ++i) {
        cout << "\n### iteration " << iteration++ << ", pose " << i << " ###" << endl;
        
        // data association
        cerr << "Testing associations ...";
        bool no_more_association = dataAssociation2(optimizer, i, xi, maxDistance);
        cerr << " done." << endl;
        
        // write output file
        writeDataFile (outputFilename, optimizer);
        
        // read data file and load to optimizer
        readDataFile (ifs, outputFilename);
        optimizer.clear();
        optimizer.load(ifs);
        ifs.close();
        
        // load robust kernel
        loadRobustKernel (robustKernel, nonSequential,  huberWidth, optimizer);

        // optimize
        optimizer.initializeOptimization();
        optimizer.optimize(maxIterations);
    }
    
    // posterior optimization loop
    cout << "\n### Posterior Optimization ###\n" << endl;
    while(true) {
        // data association
        cout << "\n### iteration " << iteration++ << " ###\n" << endl;
        cerr << "Testing associations ...";
        bool no_more_association  = dataAssociation(optimizer, xi, maxDistance);
        cerr << " done." << endl;
        
        // write output file
        writeDataFile (outputFilename, optimizer);

        // finish test
        if (no_more_association) break;

        // read data file and load to optimizer
        readDataFile (ifs, outputFilename);
        optimizer.clear();
        optimizer.load(ifs);
        ifs.close();

        // load robust kernel
        loadRobustKernel (robustKernel, nonSequential,  huberWidth, optimizer);

        // optimize
        optimizer.initializeOptimization();
        optimizer.optimize(maxIterations);
    }

    // compute time
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Elapsed time: " << elapsed_secs << " [s]" << endl;
    ProfilerStop();

    return 0;
}

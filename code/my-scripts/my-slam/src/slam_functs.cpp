#include "slam_functs.h"

void listRobustKernels (bool list) {
    if (list) {
        std::vector<std::string> kernels;
        RobustKernelFactory::instance()->fillKnownKernels(kernels);
        cout << "Robust Kernels:" << endl;
        for (size_t i = 0; i < kernels.size(); ++i) {
            cout << kernels[i] << endl;
        }
    }
}

void readDataFile (ifstream &ifs, string filename) {
    ifs.open(filename.c_str());
    if (! ifs) {
        cerr << "unable to open " << filename << endl;
    }
}

void writeDataFile (string outputFilename, SparseOptimizer &optimizer) {
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
}

void loadRobustKernel (string robustKernel, bool nonSequential, double huberWidth, SparseOptimizer &optimizer) {
    if (robustKernel.size() > 0) {
        AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator(robustKernel);
        cerr << "# Preparing robust error function ... ";
        if (creator) {
            if (nonSequential) {
                for (SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
                    SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*it);
                    if (e->vertices().size() >= 2 && std::abs(e->vertex(0)->id() - e->vertex(1)->id()) != 1) {
                        e->setRobustKernel(creator->construct());
                        if (huberWidth > 0)
                            e->robustKernel()->setDelta(huberWidth);
                    }
                }
            } else {
                for (SparseOptimizer::EdgeSet::iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
                    SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(*it);
                    e->setRobustKernel(creator->construct());
                    if (huberWidth > 0)
                        e->robustKernel()->setDelta(huberWidth);
                }
            }
            cerr << "done." << endl;
        } else {
            cerr << "Unknown Robust Kernel: " << robustKernel << endl;
        }
    }
}

void getAllPoses (SparseOptimizer &optimizer, OptimizableGraph::VertexContainer &poses) {
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    for (size_t i=0; i<vc.size(); ++i) {
        if (vc[i]->dimension() == 3) {
            poses.push_back(vc[i]);
        }
    }
}

//double getMaxDistance (SparseOptimizer &optimizer, double xi) {
//    double measInfo = 0;
//    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    
    // get measurement information
//    for (size_t i=0; i<vc.size(); ++i) {
//        if (vc[i]->dimension() == 3) {
//            set<HyperGraph::Edge*> edgeSet = vc[i]->edges();
//            for (set<HyperGraph::Edge*>::iterator it1 = edgeSet.begin(); it1 != edgeSet.end(); ++it1) {
//                // Assume landmarks are second vertex in vertexContainer
//                OptimizableGraph::Vertex* v1 = static_cast<OptimizableGraph::Vertex*> ((*it1)->vertices()[1]);
//                if (v1->dimension() == 2) {
//                    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*> (*it1);
//                    Eigen::Map<MatrixXd> info(e->informationData(), e->dimension(), e->dimension()); 
//                    measInfo = info(0,0);
//                    goto endLoops;
//                }
//            }
//        }
//    }
//    endLoops:
    
    // get max distance
//    double eta = 1 / ( 2*M_PI*(1/measInfo) );
//    double maxDistance = sqrt(log(xi/eta) / (-measInfo));
//    return maxDistance;
//}

double getMaxDistance (double xi) {
    return 1/(sqrt(2*M_PI*M_E) * xi);
}

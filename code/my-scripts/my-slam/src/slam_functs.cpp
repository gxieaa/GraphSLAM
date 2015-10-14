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
        //cout << "v" << vc[i]->id() << endl;
        if (vc[i]->dimension() == 3) {
            poses.push_back(vc[i]);
        }
    }
}

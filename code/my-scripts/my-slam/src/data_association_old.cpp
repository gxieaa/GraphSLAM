#include "data_association.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

bool data_association (SparseOptimizer& optimizer, double xi) {
    // parameters
    bool no_association = true;
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    double varDistance = get_max_var(optimizer, vc);
    //double varDistance = 5;
    bool associated[vc.size()];
    fill_n(associated, vc.size(), false);
    
    // loop through all pair of landmark poses
    for (size_t i=0; i<vc.size(); ++i) {
        OptimizableGraph::Vertex* v1 = vc[i];
        // check if vertex is landmark
        if (v1->dimension() == 2 && !associated[i]) {
            for (size_t j=i+1; j<vc.size(); ++j){
                OptimizableGraph::Vertex* v2 = vc[j];
                // check if vertex is landmark
                if (v2->dimension() == 2 && !share_pose(v1,v2) && !associated[j] && distant_test(v1, v2, varDistance)) { 
                    //cout << "testing association between (v" << v1->id() << ", " << "v" << v2->id() << ") ... " << endl;
                    if (correspondence_test(optimizer, v1, v2, xi)) {
                        // succesful association
                        make_association(v1, v2);
                        no_association = false;
                        associated[i] = true;
                        associated[j] = true;
                    }
                }
            }
        }
    }
    return no_association;
}

double get_max_var(SparseOptimizer& optimizer, OptimizableGraph::VertexContainer &vc){
    double maxVar = 0;
    for (size_t i=0; i<vc.size(); ++i) {
        OptimizableGraph::Vertex* v1 = vc[i];
        if (v1->dimension() == 2) { // check if vertex is landmark
            // computer marginal values
            std::vector<std::pair<int, int> > blockIndices;
            blockIndices.push_back(make_pair(v1->hessianIndex(), v1->hessianIndex()));
            SparseBlockMatrix<MatrixXd> spinv;
            optimizer.computeMarginals(spinv, blockIndices);
            double maxCov = fmax((*(spinv.block(v1->hessianIndex(), v1->hessianIndex())))(0,0),
                (*(spinv.block(v1->hessianIndex(), v1->hessianIndex())))(1,1));
            // replace if max variance found
            if (maxCov > maxVar) {
                maxVar = maxCov;  
            }
            
        }
    }
    cout << " (Max var: " << maxVar << ") ... ";
    return maxVar;
}

bool correspondence_test (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double xi) {
    // computer marginal values from optimizer
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
    Matrix4d marginInfoMat = marginCovMat.inverse();

    // get landmark estimate vectors
    std::vector<double> v1EstVec;
    v1->getEstimateData(v1EstVec);
    Vector2d v1Est(v1EstVec.data());
    std::vector<double> v2EstVec;
    v2->getEstimateData(v2EstVec);
    Vector2d v2Est(v2EstVec.data());
    
    // get landmark difference estimation vector
    Vector2d diffEstVec = v1Est - v2Est;

    // get landmark difference information matrix
    Matrix<double, 4, 2> diffMat;
    diffMat << Matrix2d::Identity(2,2), -1*Matrix2d::Identity(2,2);
    Matrix2d diffInfoMat = diffMat.transpose() * marginInfoMat * diffMat;

    // get likelihood
    double likelihood;
    likelihood = pow((2*M_PI * diffInfoMat.inverse()).determinant(), -0.5) * exp(-0.5 * diffEstVec.transpose() * diffInfoMat * diffEstVec);
    //cout << "likelihood (v" << v1->id() << ", " << "v" << v2->id() << "): ";
    //cout << likelihood << endl;
    return likelihood > xi;
}

bool share_pose(OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2) {
    //cout << "test share pose between (v" << v1->id() << ", " << "v" << v2->id() << ")";
    set<HyperGraph::Edge*> edgeSetV1 = v1->edges();
    set<HyperGraph::Edge*> edgeSetV2 = v2->edges();
    for (set<HyperGraph::Edge*>::iterator it1 = edgeSetV1.begin(); it1 != edgeSetV1.end(); ++it1) {
        // Assume poses are first vertex in vertexContainer
        HyperGraph::Vertex* v1Pose = (*it1)->vertices()[0];
        for (set<HyperGraph::Edge*>::iterator it2 = edgeSetV2.begin(); it2 != edgeSetV2.end(); ++it2) {
            HyperGraph::Vertex* v2Pose = (*it2)->vertices()[0];
            if (v1Pose->id() == v2Pose->id()) {
                //cout << "(v" << v1->id() << ", " << "v" << v2->id() << ") share pose" << endl;
                return true;
            }
        }
    }
    return false;
}

bool distant_test(OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double varDistance) {
    std::vector<double> v1EstVec;
    v1->getEstimateData(v1EstVec);
    Vector2d v1Est(v1EstVec.data());
    std::vector<double> v2EstVec;
    v2->getEstimateData(v2EstVec);
    Vector2d v2Est(v2EstVec.data());
    return (v1Est - v2Est).norm() < varDistance;
}

void make_association (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2) {
    //cout << "Association found between (v" << v1->id() << ", " << "v" << v2->id() << ")" << endl;
    v2->setId(v1->id());
}

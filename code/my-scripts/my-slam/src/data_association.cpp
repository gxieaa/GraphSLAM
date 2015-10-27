#include "data_association.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

bool dataAssociation (SparseOptimizer& optimizer, double xi, double maxDistance) {
    // parameters
    bool noAssociation = true;
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    //double varDistance = getMaxVar(optimizer, vc);
    //double varDistance = 5;
    bool associated[vc.size()];
    fill_n(associated, vc.size(), false);
    
    // loop through all pair of landmark poses
    for (size_t i=0; i<vc.size(); ++i) {
        OptimizableGraph::Vertex* v1 = vc[i];
        // check if vertex is landmark
        if (v1->dimension() == 2) {
        //if (v1->dimension() == 2 && !associated[i]) {
            for (size_t j=i+1; j<vc.size(); ++j){
                OptimizableGraph::Vertex* v2 = vc[j];
                // check if vertex is landmark
                //if (v2->dimension() == 2 && !sharePose(v1,v2)) {
                if (v2->dimension() == 2 && !sharePose(v1,v2) && distantTest(v1, v2, maxDistance)) {                
                //if (v2->dimension() == 2 && !sharePose(v1,v2) && !associated[j] && distantTest(v1, v2, maxDistance)) { 
                    //cout << "testing association between (v" << v1->id() << ", " << "v" << v2->id() << ") ... " << endl;
                    if (correspondenceTest(optimizer, v1, v2, xi)) {
                        // succesful association
                        makeAssociation(v1, v2);
                        noAssociation = false;
                        associated[i] = true;
                        associated[j] = true;
                        //goto leaveAssociation;
                    }
                }
            }
        }
    }
    leaveAssociation:
    return noAssociation;
}

bool dataAssociation2 (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance) {
    // parameters
    bool noAssociation = true;
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    OptimizableGraph::Vertex* currentPose = vc[poseIndex];
    set<HyperGraph::Edge*> edgeSetCurr = currentPose->edges();
    int minIndCurrVertex = getMinInd(edgeSetCurr, vc);
    bool associated[vc.size()];
    fill_n(associated, vc.size(), false);
    //cout << "pose id: " << currentPose->id() << endl;
    
    // for all landmarks observed in current pose
    for (set<HyperGraph::Edge*>::iterator it1 = edgeSetCurr.begin(); it1 != edgeSetCurr.end(); ++it1) {
        // Assume landmarks are second vertex in vertexContainer
        OptimizableGraph::Vertex* v1 = static_cast<OptimizableGraph::Vertex*> ((*it1)->vertices()[1]);
        if (v1->dimension() == 2) {
            //cout << "current pose masurement id: " << v1->id() << endl; 
            // for all landmarks of the past
            int i2 = minIndCurrVertex - 1;
            OptimizableGraph::Vertex* v2 = vc[i2];
            while (v2->dimension() == 2) {
                //cout << "testing association between (v" << v1->id() << ", " << "v" << v2->id() << ") ... " << endl;
                if (!associated[i2]){
                    if (distantTest(v1, v2, maxDistance)){
                        if (correspondenceTest(optimizer, v1, v2, xi)) {
                            makeAssociation(v1, v2);
                            noAssociation = false;
                            associated[i2] = true;
                            break;
                        }
                    }
                }
                --i2; 
                v2 = vc[i2];
                //break;
            }
        }
    }
    return noAssociation;
}

int getMinInd (set<HyperGraph::Edge*> &es, OptimizableGraph::VertexContainer &vc) {
    int minID = INT_MAX;
    for (set<HyperGraph::Edge*>::iterator it1 = es.begin(); it1 != es.end(); ++it1) {
        // Assume landmarks are second vertex in vertexContainer
        OptimizableGraph::Vertex* v1 = static_cast<OptimizableGraph::Vertex*> ((*it1)->vertices()[1]);
        if (v1->dimension() == 2 && v1->id() < minID) minID = v1->id();
    }
    int minInd = 0;
    for (int i=0; i<vc.size(); ++i){
        if (vc[i]->id() == minID) {
            minInd = i;
            break;
        }
    }
    return minInd;
}

double getMaxVar(SparseOptimizer &optimizer, OptimizableGraph::VertexContainer &vc){
    double maxVar = 0;
    for (size_t i=0; i<vc.size(); ++i) {
        OptimizableGraph::Vertex* v1 = vc[i];
        if (v1->dimension() == 2) { // check if vertex is landmark
            // computer marginal values
            double maxCov = getMaxVarLandmark(optimizer, v1);
            // replace if max variance found
            if (maxCov > maxVar) {
                maxVar = maxCov;  
            }
            
        }
    }
    cout << " (Max var: " << maxVar << ") ... ";
    return maxVar;
}

double getMaxVarLandmark (SparseOptimizer &optimizer, OptimizableGraph::Vertex* v1) {
    // computer marginal values
    std::vector<std::pair<int, int> > blockIndices;
    blockIndices.push_back(make_pair(v1->hessianIndex(), v1->hessianIndex()));
    SparseBlockMatrix<MatrixXd> spinv;
    optimizer.computeMarginals(spinv, blockIndices);
    double maxCov = fmax((*(spinv.block(v1->hessianIndex(), v1->hessianIndex())))(0,0),
        (*(spinv.block(v1->hessianIndex(), v1->hessianIndex())))(1,1));
    return maxCov;
}

bool correspondenceTest (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double xi) {
    // computer marginal values from optimizer
    std::vector<std::pair<int, int> > blockIndices;
    blockIndices.push_back(make_pair(v1->hessianIndex(), v1->hessianIndex()));
    blockIndices.push_back(make_pair(v1->hessianIndex(), v2->hessianIndex()));
    blockIndices.push_back(make_pair(v2->hessianIndex(), v1->hessianIndex()));
    blockIndices.push_back(make_pair(v2->hessianIndex(), v2->hessianIndex()));
    SparseBlockMatrix<MatrixXd> spinv;
    //cout << "computeMarginals!" << endl;
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

bool sharePose(OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2) {
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

bool distantTest(OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double varDistance) {
    std::vector<double> v1EstVec;
    v1->getEstimateData(v1EstVec);
    Vector2d v1Est(v1EstVec.data());
    std::vector<double> v2EstVec;
    v2->getEstimateData(v2EstVec);
    Vector2d v2Est(v2EstVec.data());
    return (v1Est - v2Est).norm() < varDistance;
}

void makeAssociation (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2) {
    //cout << "Association found between (v" << v1->id() << ", " << "v" << v2->id() << ")" << endl;
    v2->setId(v1->id());
}

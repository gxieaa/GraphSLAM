#include "data_association.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

bool incDataAssociation (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance) {
    // parameters
    bool noAssociation = true;
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    OptimizableGraph::Vertex* currentPose = vc[poseIndex];
    set<HyperGraph::Edge*> edgeSetCurr = currentPose->edges();
    vector<int> associated;
    associated.clear();
    
    // for all landmarks observed in current pose
    for (set<HyperGraph::Edge*>::iterator it1 = edgeSetCurr.begin(); it1 != edgeSetCurr.end(); ++it1) {
        // Assume landmarks are second vertex in vertexContainer
        OptimizableGraph::Vertex* v1 = static_cast<OptimizableGraph::Vertex*> ((*it1)->vertices()[1]);
        if (v1->dimension() == 2) {
            // for all landmarks of the past
            for (int i = poseIndex-1; i>=0; --i) {
		OptimizableGraph::Vertex* pastPose = vc[i];
                set<HyperGraph::Edge*> edgeSetPast = pastPose->edges();
                for (set<HyperGraph::Edge*>::iterator it2 = edgeSetPast.begin(); it2 != edgeSetPast.end(); ++it2) {
                    // Assume landmarks are second vertex in vertexContainer
                    OptimizableGraph::Vertex* v2 = static_cast<OptimizableGraph::Vertex*> ((*it2)->vertices()[1]);
                    if (v2->dimension() == 2 && v2->id() != v1->id()) {
                        if (find(associated.begin(), associated.end(), v2->id()) == associated.end()) {
                            if (distantTest(v1, v2, maxDistance)) {
                                if (xi <= 0 || correspondenceTest(optimizer, v1, v2, xi)) {
                                    // association successful
                                    //cout << "asso: " << v1->id() << ", " << v2->id() << endl;
				    optimizer.mergeVertices(v1, v2, false);
                                    noAssociation = false;
                                    associated.push_back(v2->id());
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return noAssociation;
}

bool fullDataAssociation (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance) {
    // parameters
    bool noAssociation = true;
    
    // for all past poses
    for (int i = poseIndex; i>=0; --i) {
        //cout << "pose : " << i << endl;
	// test association
        noAssociation = noAssociation && incDataAssociation (optimizer, i, xi, maxDistance);
    }
    return noAssociation;
}

bool correspondenceTest (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double xi) {
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
    return likelihood > xi;
}

bool distantTest(OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double maxDistance) {
    std::vector<double> v1EstVec;
    v1->getEstimateData(v1EstVec);
    Vector2d v1Est(v1EstVec.data());
    std::vector<double> v2EstVec;
    v2->getEstimateData(v2EstVec);
    Vector2d v2Est(v2EstVec.data());
    return (v1Est - v2Est).norm() < maxDistance;
}

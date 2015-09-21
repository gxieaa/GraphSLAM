#include "data_association.h"

const double xi = 10;

void data_association (SparseOptimizer& optimizer) {
    OptimizableGraph::VertexContainer vc = optimizer.activeVertices();
    for (size_t i=0; i<vc.size(); ++i) {
        OptimizableGraph::Vertex* v1 = vc[i];
        if (v1->dimension() == 2) { // check if vertex is landmark
            for (size_t j=i+1; j<vc.size(); ++j){
                OptimizableGraph::Vertex* v2 = vc[j];
                if (v2->dimension() == 2) { // check if vertex is landmark
                    if (correspondence_test(optimizer, &v1, &v2) > xi) {
                        // succesful association
                        make_association(v1, v2);
                    }
                }
            }
        }
    }
}

bool correspondence_test (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2) {
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
    //cout << "likelihood (v" << v1->id() << ", " << "v" << v2->id() << "):" << endl;
    //cout << likelihood << endl;
    return likelihood > xi;
}

void make_association(OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2){
    cout << "Association found between (v" << v1->id() << ", " << "v" << v2->id() << ") with likelhood:" << endl;
    cout << likelihood << endl;
}

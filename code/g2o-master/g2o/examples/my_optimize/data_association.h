#include <iostream>

const double xi = 10;

void data_association (SparseOptimizer& optimizer);

bool correspondence_test (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);

void make_association(OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2);

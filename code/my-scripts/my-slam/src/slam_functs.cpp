#include "slam_functs.h"

using namespace std;
using namespace g2o;

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

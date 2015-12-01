# GraphSLAM

A GraphSLAM implementation using g2o framework

### Requirements

g2o framework ([link](https://github.com/RainerKuemmerle/g2o)) installed is required to compile the C++ scripts. Read theirs README to check g2o requirements. After that you can safely compile each script individually by usual:

- `mkdir build`
- `cd build`
- `cmake ../`
- `make` 

### Directories:
- code:
    - my-scripts: C++ scripts that performs SLAM magic
    - python-helpers: Python scripts that make easy to run GraphSLAM test
    - gprof2dot: Module to generate profiler graph (for performace test)
- doc:
    - Memoria: Memoria (tesis) document explaining the work

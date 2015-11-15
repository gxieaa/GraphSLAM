pprof --callgrind ../../my-scripts/my-slam/build/my_slam ./my_slam_prof.prof > slam_prof.callgrind
#kcachegrind ls.callgrind
../../gprof2dot/gprof2dot/./gprof2dot.py --format=callgrind --output=slam_prof.dot slam_prof.callgrind
dot -Tpng slam_prof.dot -o slam_prof.png

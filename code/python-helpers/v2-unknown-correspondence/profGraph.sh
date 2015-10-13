pprof --callgrind ../../my-scripts/my-slam/build/my_slam /home/francocurotto/pprof/my_slam_prof.prof > ls.callgrind
kcachegrind ls.callgrind
../../../../gprof2dot/./gprof2dot.py --format=callgrind --output=out.dot ls.callgrind
dot -Tpng out.dot -o graph.png

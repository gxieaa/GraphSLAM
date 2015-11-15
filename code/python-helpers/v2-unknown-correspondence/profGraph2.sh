pprof --callgrind ../../my-scripts/my-slam/build/my_slam2 ./my_slam2_prof.prof > slam2_prof.callgrind
#kcachegrind ls.callgrind
../../gprof2dot/gprof2dot/./gprof2dot.py --format=callgrind --output=slam2_prof.dot slam2_prof.callgrind
dot -Tpng slam2_prof.dot -o slam2_prof.png

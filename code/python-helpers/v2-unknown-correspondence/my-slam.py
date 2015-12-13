# imports
import subprocess
import sys
import time
sys.path.append('../commons')
from slamFunctions import *
from associationFunctions import *

# variables
g2oIterations = 20
xi = 0
nlandmarks = 30
infoOdomPos = 1000
infoOdomAng = 10000
infoPointSen = 1000
dataSkip = 1
interOpt = 400
simSteps = 400
disTest = 1
kernelWidth = 1
poseSkip = 400

# compile
buildPath = "../../my-scripts/my-slam/build/"
subprocess.call(["make", "-C", buildPath])

# run g2o tests
start_time = time.time()

# paths and filenames
binSimPath = "../../my-scripts/my-simulator/build/"
suffix = "_it_" + str(g2oIterations)  + "_xi_" + str(xi) + "_nl_" + str(nlandmarks) + "_op_" + str(infoOdomPos) + "_oa_" + str(infoOdomAng) + "_lp_" + str(infoPointSen) + "_dsk_" + str(dataSkip) + "_io_"  + str(interOpt) + "_ds_" + str(simSteps) + "_dt_" + str(disTest) + "_kw_" + str(kernelWidth) + "_ps_" + str(poseSkip)
simPath = "data/sim_out" + suffix + ".g2o"
guessInPath = "data/guess_in" + suffix + ".g2o"
guessOutPath = "data/guess_out" + suffix + ".g2o"
anonOutPath = "data/anon_out" + suffix + ".g2o"
optPath = "res/opt_out" + suffix + ".g2o"
figPath = "res/res"

# get simulation data
subprocess.call([binSimPath+"./my_simulator", "-hasOdom", "-hasPointSensor",
                 "-nlandmarks", str(nlandmarks), "-simSteps", str(simSteps),
                 "-infoOdomPos", str(infoOdomPos), "-infoOdomAng", str(infoOdomAng),
                 "-infoPointSen", str(infoPointSen), simPath])
fixNode(simPath, 1000+nlandmarks)

# get initial guess
getInitialGuess(simPath, guessInPath)

# anonymize landmarks
anonymizeLandmarks(guessInPath, anonOutPath)
subprocess.call(["g2o", "-i", "0", "-guessOdometry",
                 "-o", guessOutPath, anonOutPath])

# make optimization
subprocess.call(["env", "CPUPROFILE=./my_slam_prof.prof",
                buildPath+"./my_slam",
                "-i", str(g2oIterations),
                "-t", str(xi),
                "-robustKernel", "Huber",
                "-robustKernelWidth", str(kernelWidth),
                "-poseSkip", str(poseSkip),
                "-interOpt", str(interOpt),
                "-disTest", str(disTest),
                "-o", optPath, anonOutPath])

# plot results
plotResults(simPath, guessOutPath, optPath, figPath, suffix)

# compute elapsed time
elapsed_time = time.time() - start_time
print "Total time tests: " + str(elapsed_time) + " [s]"

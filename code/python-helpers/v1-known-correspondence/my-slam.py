# imports
import subprocess
import sys
import time
sys.path.append('../commons')
from slamFunctions import *

# variables
g2oIterations = 20
nlandmarks = 30
simSteps = 400
infoOdomPos = 1
infoOdomAng = 10
infoPointSen = 1
kernelWidth = 1

# run g2o tests
start_time = time.time()

# paths and filenames
binSimPath = "../../my-scripts/my-simulator/build/"
suffix = "_it_" + str(g2oIterations)  + "_nl_" + str(nlandmarks) + "_op_" + str(infoOdomPos) + "_oa_" + str(infoOdomAng) + "_lp_" + str(infoPointSen) + "_ds_" + str(simSteps) + "_kw_" + str(kernelWidth)
simPath = "data/sim_out" + suffix + ".g2o"
guessInPath = "data/guess_in" + suffix + ".g2o"
guessOutPath = "data/guess_out" + suffix + ".g2o"
optPath = "res/opt_out" + suffix + ".g2o"
figPath = "res/res"

# get simulation data
subprocess.call([binSimPath+"./my_simulator",
                 "-hasOdom", "-hasPointSensor",
                 "-nlandmarks", str(nlandmarks), "-simSteps", str(simSteps),
                 "-infoOdomPos", str(infoOdomPos), "-infoOdomAng", str(infoOdomAng),
                 "-infoPointSen", str(infoPointSen), simPath])
fixNode(simPath, 1000+nlandmarks)

# get initial guess
getInitialGuess(simPath, guessInPath)
subprocess.call(["g2o", "-i", "0", "-guessOdometry",
                 "-o", guessOutPath, guessInPath])

# make optimization
subprocess.call(["g2o", "-i", str(g2oIterations), "-guessOdometry",
                 #"-inc",
                 "-robustKernel", "Huber",
                "-robustKernelWidth", str(kernelWidth),
                 #"-solver",
                 "-o", optPath, guessInPath])

# compute elapsed time
elapsed_time = time.time() - start_time
print "Total time tests: " + str(elapsed_time) + " [s]"

# plot results
plotGuess(simPath, guessOutPath, "res/guess"+ suffix)
plotResults(simPath, guessOutPath, optPath, figPath, suffix)

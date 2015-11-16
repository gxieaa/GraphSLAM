# imports
import subprocess
import sys
import time
sys.path.append('../commons')
from slamFunctions import *
from associationFunctions import *

def main():
    # variables
    g2oIterations = 10
    xi = 1e-200
    nlandmarks = 30
    simSteps = 400
    infoOdomPos = 1000
    infoOdomAng = 1000
    infoPointSen = 1000
    interOpt = 50
    
    # compile
    buildPath = "../../my-scripts/my-slam-final/build/"
    subprocess.call(["make", "-C", buildPath]) 
    
    # run g2o tests
    start_time = time.time()
    
    # paths and filenames
    binSimPath = "../../my-scripts/my-simulator/build/"
    binOptPath = "../../my-scripts/my-slam-final/build/"
    suffix = "_xi_" + str(xi) + "_op_" + str(infoOdomPos) + "_oa_" + str(infoOdomAng) + "_lp_" + str(infoPointSen) + "_ds_" + str(simSteps)
    simPath = "data/sim_out" + suffix + ".g2o"
    guessInPath = "data/guess_in" + suffix + ".g2o"
    guessOutPath = "data/guess_out" + suffix + ".g2o"
    anonOutPath = "data/anon_out" + suffix + ".g2o"
    optPath = "res/opt_out" + suffix + ".g2o"
    figPath = "res/res"+ suffix

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
                    binOptPath+"./my_slam", 
                    "-i", str(g2oIterations), 
                    "-t", str(xi),
                    #"-robustKernel", "Huber",
                    #"-robustKernelWidth", str(1),
                    #"-poseSkip", str(1),
                    "-interOpt", str(interOpt),
                    "-o", optPath, anonOutPath])
                     
    # plot results
    plotResults(simPath, guessOutPath, optPath, figPath, None, None)
    
    # compute elapsed time
    elapsed_time = time.time() - start_time
    print "Total time tests: " + str(elapsed_time) + " [s]"
    
if __name__ == '__main__':
    main()

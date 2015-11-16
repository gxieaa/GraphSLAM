# imports
import subprocess
import sys
sys.path.append('../commons')
from slamFunctions import *

def main():
    # variables
    g2oIterations = 5
    nlandmarks = 300
    simSteps = 300
    infoOdomPos = 1600
    infoOdomAng = 5000
    infoPointSen = 100
    
    # paths and filenames
    binSimPath = "../../my-scripts/my-simulator/build/"
    suffix = "_op_" + str(infoOdomPos) + "_oa_" + str(infoOdomAng) + "_lp_" + str(infoPointSen) + "_ds_" + str(simSteps) 
    simPath = "data/sim_out" + suffix + ".g2o"
    guessInPath = "data/guess_in" + suffix + ".g2o"
    guessOutPath = "data/guess_out" + suffix + ".g2o"
    optPath = "res/opt_out" + suffix + ".g2o"
    figPath = "res/res"+ suffix

    # get simulation data
    subprocess.call([binSimPath+"./my_simulator",
                     "-hasOdom", "-hasPointSensor",
                     "-nlandmarks", str(nlandmarks), "-simSteps", str(simSteps),
                     "-infoOdomPos", str(infoOdomPos), "-infoOdomAng", str(infoOdomAng),
                     "-infoPointSen", str(infoPointSen), simPath])
    fixNode(simPath, 1000+simSteps)
                     
    # get initial guess
    getInitialGuess(simPath, guessInPath)
    subprocess.call(["g2o", "-i", "0", "-guessOdometry",
                     "-o", guessOutPath, guessInPath])

    # make optimization
    subprocess.call(["g2o", "-i", str(g2oIterations), "-guessOdometry",
                     #"-inc",
                     "-robustKernel", "Huber",
                    "-robustKernelWidth", str(1),
                     #"-solver",
                     "-o", optPath, guessInPath])
                     
    # plot results
    plotResults(simPath, guessOutPath, optPath, figPath)
    
if __name__ == '__main__':
    main()

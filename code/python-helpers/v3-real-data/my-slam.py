    #imports
import subprocess
import sys
import time
import os
sys.path.append('../commons')
sys.path.append('data/Parque OHiggins')
sys.path.append('data/Parque OHiggins 2')
sys.path.append('data/Victoria Park Felipe')
from slamFunctions import *
from OHigginsRaw2g2o import *
from OHiggins2Raw2g2o import *
from victoriaRaw2g2o import *

def main():
    
    # variables
    g2oIterations = 10
    xi = 1e-100
    kernelWidth = 1
    infoOdomPos = 1000
    infoOdomAng = 1000
    infoPointSen = 1000
    poseSkip = 1
    dataSkip = 10
    interOpt = 50
    dataSize = 500
    
    # compile
    buildPath = "../../my-scripts/my-slam/build/"
    subprocess.call(["make", "-C", buildPath]) 
    
    # directory generator
    makeDirs(["res",])
    
    # run g2o tests
    start_time = time.time()
    runG2O(g2oIterations, xi, kernelWidth, infoOdomPos, infoOdomAng, infoPointSen, poseSkip, dataSkip, interOpt, dataSize)
    elapsed_time = time.time() - start_time
    print "Total time tests: " + str(elapsed_time) + " [s]"
    
def runG2O(g2oIterations, xi, kernelWidth, infoOdomPos, infoOdomAng, infoPointSen, poseSkip, dataSkip, interOpt, dataSize):
    
    # paths
    binOptPath = "../../my-scripts/my-slam/build/"
    dataPath = "data/Parque OHiggins/ohiggins.g2o"
    #dataPath = "data/Parque OHiggins 2/ohiggins2.g2o"
    #dataPath = "data/Victoria Park Felipe/victoria.g2o"
    dataName = os.path.splitext(os.path.basename(dataPath))[0]
    dataDir = os.path.dirname(dataPath) + "/"
    guessOutPath = "res/guess_out_" + dataName + ".g2o"
    resPath = "res/opt_out_" + dataName + ".g2o"
    figPath = "res/res_" + dataName
    
    # generate g2o format data
    print "Generating data in g2o format"
    if dataName == "ohiggins":
        ohigginsRaw2g2o(infoOdomPos, infoOdomAng, infoPointSen, dataDir, dataSkip, dataSize)
    if dataName == "ohiggins2":
        ohiggins2Raw2g2o(infoOdomPos, infoOdomAng, infoPointSen, dataDir, dataSkip, dataSize)
    elif dataName == "victoria":
        victoriaRaw2g2o(infoOdomPos, infoOdomAng, infoPointSen, dataDir, dataSkip, dataSize)
    
    
    # get initial guess
    subprocess.call(["g2o", "-i", "0", "-guessOdometry",
                     "-o", guessOutPath, dataPath])
                     
    # optimize
    
    print "Optimize"
    subprocess.call(["env", "CPUPROFILE=./my_slam2_prof.prof",
                    binOptPath+"./my_slam2", 
                    "-i", str(g2oIterations), 
                    "-t", str(xi),
                    "-robustKernel", "Huber",
                    "-robustKernelWidth", str(kernelWidth),
                    "-poseSkip", str(poseSkip),
                    "-interOpt", str(interOpt),
                    "-o", resPath, guessOutPath])
            
    # plot results
    #sufix = "_opt"
    sufix = "_xi_" + str(xi) + "_op_" + str(infoOdomPos) + "_oa_" + str(infoOdomAng) + "_lp_" + str(infoPointSen) + "_ps_" + str(poseSkip)
    #makeRealPlots(dataPath, figPath) 
    #makeRealPlots(guessOutPath, figPath, "_odom")
    #makeRealPlots(resPath, figPath, sufix)
    plotResults(dataDir+"gt.g2o", guessOutPath, resPath, figPath, 0, 0)
     
if __name__ == '__main__':
    main()

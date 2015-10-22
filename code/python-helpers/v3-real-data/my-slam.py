#imports
import subprocess
import sys
import time
import os
sys.path.append('../commons')
sys.path.append('data/Parque OHiggins')
sys.path.append('data/Parque OHiggins Test')
sys.path.append('data/Victoria Park Felipe')
from slamFunctions import *
from OHigginsRaw2g2o import *
from victoriaRaw2g2o import *

def main():
    
    # variables
    g2oIterations = 10
    xi = 0.0001
    kernelWidth = 1
    dataInfo = 100
    
    # compile
    buildPath = "../../my-scripts/my-slam/build/"
    subprocess.call(["make", "-C", buildPath]) 
    
    # run g2o tests
    start_time = time.time()
    runG2O(g2oIterations, xi, kernelWidth, dataInfo)
    elapsed_time = time.time() - start_time
    print "Total time tests: " + str(elapsed_time) + " [s]"
    
def runG2O(g2oIterations, xi, kernelWidth, dataInfo):
    
    # paths
    binOptPath = "../../my-scripts/my-slam/build/"
    #dataPath = "data/Parque OHiggins/ohiggins.g2o"
    dataPath = "data/Parque OHiggins Test/ohiggins.g2o"
    #dataPath = "data/Victoria Park Felipe/victoria.g2o"
    dataName = os.path.splitext(os.path.basename(dataPath))[0]
    dataDir = os.path.dirname(dataPath) + "/"
    guessOutPath = "res/guess_out_" + dataName + ".g2o"
    resPath = "res/opt_out_" + dataName + ".g2o"
    figPath = "res/res_" + dataName
    
    # generate g2o format data
    print "generating data in g2o format"
    if dataName == "ohiggins":
        ohigginsRaw2g2o(dataInfo, dataDir)
    else:
        victoriaRaw2g2o(dataInfo, dataDir)
    
    
    # get initial guess
    subprocess.call(["g2o", "-i", "0", "-guessOdometry",
                     "-o", guessOutPath, dataPath])
                     
    # optimize
    print "optimize"
    subprocess.call(["env", "CPUPROFILE=./my_slam2_prof.prof",
                    binOptPath+"./my_slam2", 
                    "-i", str(g2oIterations), 
                    "-t", str(xi),
                    "-robustKernel", "Huber",
                    "-robustKernelWidth", str(kernelWidth),
                    "-o", resPath, guessOutPath])
                     
    # plot results
    sufix = "_xi_" + str(xi) + "_di_" + str(dataInfo)
    #makeRealPlots(dataPath, figPath) 
    #makeRealPlots(guessOutPath, figPath, "_odom")
    makeRealPlots(resPath, figPath,sufix)
     
if __name__ == '__main__':
    main()

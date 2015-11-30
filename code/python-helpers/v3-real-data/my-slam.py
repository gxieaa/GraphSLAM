
#imports
import subprocess
import sys
import time
import os
from time import gmtime, strftime
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
    g2oIterations = 20
    xi = 0.001
    infoOdomPos = 1600
    infoOdomAng = 20000
    infoPointSen = 5
    dataSkip = 5
    interOpt = 500
    dataSize = 2000
    disTest = 0
    kernelWidth = 1
    poseSkip = 1
    
    # compile
    buildPath = "../../my-scripts/my-slam/build/"
    subprocess.call(["make", "-C", buildPath]) 
    
    # run g2o tests
    start_time = time.time()
    
    # paths
    #dataPath = "data/Parque OHiggins/ohiggins.g2o"
    #dataPath = "data/Parque OHiggins 2/ohiggins2.g2o"
    dataPath = "data/Victoria Park Felipe/victoria.g2o"
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
    subprocess.call(["env", "CPUPROFILE=./my_slam_prof.prof",
                    buildPath+"./my_slam", 
                    "-i", str(g2oIterations), 
                    "-t", str(xi),
                    "-robustKernel", "Huber",
                    "-robustKernelWidth", str(kernelWidth),
                    "-poseSkip", str(poseSkip),
                    "-interOpt", str(interOpt),
                    "-disTest", str(disTest),
                    "-o", resPath, guessOutPath])
        
    # plot results
    currTime = strftime("_%Y-%m-%d %H:%M:%S", gmtime())
    suffix = "_it_" + str(g2oIterations)  + "_xi_" + str(xi) + "_op_" + str(infoOdomPos) + "_oa_" + str(infoOdomAng) + "_lp_" + str(infoPointSen) + "_dsk_" + str(dataSkip) + "_io_"  + str(interOpt) + "_ds_" + str(dataSize) + "_dt_" + str(disTest) + "_kw_" + str(kernelWidth) + "_ps_" + str(poseSkip)
    #makeRealPlots(guessOutPath, guessOutPath, figPath, "_odom")
    makeRealPlots(guessOutPath, resPath, figPath, currTime + suffix)
    #plotResults(dataDir+"gt.g2o", guessOutPath, resPath, figPath)
    
    # compute elapsed time
    elapsed_time = time.time() - start_time
    print "Total time tests: " + str(elapsed_time) + " [s]"
     
if __name__ == '__main__':
    main()

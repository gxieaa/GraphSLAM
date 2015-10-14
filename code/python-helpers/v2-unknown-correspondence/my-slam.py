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
    nlandmarks = 100
    simSteps = 100
    #infoOdomPos = [500, 100, 50, 10, 5]
    #infoOdomAng = [5000, 1000, 500, 100, 50]
    #infoPointSen = [1000, 500, 100, 50, 10]
    infoOdomPos = [5000]
    infoOdomAng = [5000]
    infoPointSen = [5000]
    
    n = len(infoOdomPos)
    
    # compile
    buildPath = "../../my-scripts/my-slam/build/"
    subprocess.call(["make", "-C", buildPath]) 
    
    # directory generator
    dirNames = ['sim_out', 'guess_in', 'guess_out', 'opt_out', 'figs', 'anon_out']
    makeDirs(dirNames)
    
    # run g2o tests
    start_time = time.time()
    for i in range(n):
        runG2O(dirNames, g2oIterations, nlandmarks, simSteps, infoOdomPos[i], infoOdomAng[i], infoPointSen[i])
        
    elapsed_time = time.time() - start_time
    print "Total time tests: " + str(elapsed_time) + " [s]"
    
    # profiler
    #subprocess.call(["pprof", "--callgrind", "../../my-scripts/my-slam/build/my_slam",
                     #"/home/francocurotto/pprof/my_slam_prof.prof", ">", "ls.callgrind"])
    #subprocess.call(["kcachegrind", "ls.callgrind"])
    
    # pprof --callgrind ../../my-scripts/my-slam/build/my_slam /home/francocurotto/pprof/my_slam_prof.prof > ls.callgrind
    # kcachegrind ls.callgrind
    # ../../../../gprof2dot/./gprof2dot.py --format=callgrind --output=out.dot ls.callgrind
    # dot -Tpng out.dot -o graph.png
def runG2O(dirNames, g2oIterations, nlandmarks, simSteps, infoOdomPos, infoOdomAng, infoPointSen):
    
    # paths and filenames
    binSimPath = "../../my-scripts/my-simulator/build/"
    binOptPath = "../../my-scripts/my-slam/build/"
    parameters = [g2oIterations, nlandmarks, simSteps, infoOdomPos, infoOdomAng, infoPointSen]
    paramNames = ["i", "s", "l", "p", "a", "ps"] 
    simFilename = genFilename(dirNames[0]+"/sim_out", paramNames, parameters,".g2o")
    guessInFilename = genFilename(dirNames[1]+"/guess_in", paramNames, parameters,".g2o")
    guessOutFilename = genFilename(dirNames[2]+"/guess_out", paramNames, parameters,".g2o")
    anonOutFilename = genFilename(dirNames[5]+"/anon_out", paramNames, parameters,".g2o")
    optFilename = genFilename(dirNames[3]+"/opt_out", paramNames, parameters,".g2o")
    optFilename2 = genFilename(dirNames[3]+"/opt_out2", paramNames, parameters,".g2o")
    figFilename = genFilename(dirNames[4]+"/res", paramNames, parameters,"")
    figFilename2 = genFilename(dirNames[4]+"/res2", paramNames, parameters,"")

    # get simulation data
    subprocess.call([binSimPath+"./my_simulator", "-hasOdom", "-hasPointSensor",
                     "-nlandmarks", str(nlandmarks), "-simSteps", str(simSteps),
                     "-infoOdomPos", str(infoOdomPos), "-infoOdomAng", str(infoOdomAng),
                     "-infoPointSen", str(infoPointSen), simFilename])
    fixNode(simFilename, 1000+simSteps)
    
    # get initial guess
    getInitialGuess(simFilename, guessInFilename)
    
    # anonymize landmarks
    anonymizeLandmarks(guessInFilename, anonOutFilename)
    subprocess.call(["g2o", "-i", "0", "-guessOdometry",
                     "-o", guessOutFilename, anonOutFilename])
    
    ### SLAM 2 ## 
    # make optimization
    
    subprocess.call(["env", "CPUPROFILE=/home/francocurotto/pprof/my_slam_prof.prof",
                    binOptPath+"./my_slam2", 
                    "-i", str(g2oIterations), 
                    "-t", str(0.1),
                    #"-robustKernel", "Huber",
                    #"-robustKernelWidth", str(1),
                    "-o", optFilename2, anonOutFilename])
                     
    # plot results
    plotResults(simFilename, guessOutFilename, optFilename2, figFilename2, [-15, 20], [-20, 20])
    
    
    ### SLAM 1 ## 
    # make optimization
    '''
    subprocess.call(["env", "CPUPROFILE=/home/francocurotto/pprof/my_slam_prof.prof",
                    binOptPath+"./my_slam", 
                    "-i", str(g2oIterations), 
                    "-t", str(0.1),
                    #"-robustKernel", "Huber",
                    #"-robustKernelWidth", str(1),
                    "-o", optFilename, anonOutFilename])
                     
    # plot results
    plotResults(simFilename, guessOutFilename, optFilename, figFilename, [-15, 20], [-20, 20])
    '''
    
if __name__ == '__main__':
    main()

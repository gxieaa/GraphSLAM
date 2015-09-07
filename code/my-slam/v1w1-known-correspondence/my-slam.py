# imports
import subprocess
import sys
sys.path.append('../commons')
from slamFunctions import *

def main():
    # variables
    #g2oIterations = 10
    #nlandmarks = 100
    #simSteps = 100
    #infoOdomPos = 500
    #infoOdomAng = 5000
    #infoPointSen = 1000
    g2oIterations = 10
    nlandmarks = 300
    simSteps = 300
    infoOdomPos = [500, 100, 50, 10, 5]
    infoOdomAng = [5000, 1000, 500, 100, 50]
    infoPointSen = [1000, 500, 100, 50, 10]
    n = len(infoOdomPos)
    
    # directory generator
    dirNames = ['sim_out', 'guess_in', 'guess_out', 'opt_out', 'figs']
    makeDirs(dirNames)
    
    for i in range(n):
        runG2O(dirNames, g2oIterations, nlandmarks, simSteps, infoOdomPos[i], infoOdomAng[i], infoPointSen[i])
    
def runG2O(dirNames, g2oIterations, nlandmarks, simSteps, infoOdomPos, infoOdomAng, infoPointSen):
    
    # paths and filenames
    parameters = [g2oIterations, nlandmarks, simSteps, infoOdomPos, infoOdomAng, infoPointSen]
    paramNames = ["i", "s", "l", "p", "a", "ps"] 
    binPath = "../../g2o-master/bin/"
    simFilename = genFilename(dirNames[0]+"/sim_out", paramNames, parameters,".g2o")
    guessInFilename = genFilename(dirNames[1]+"/guess_in", paramNames, parameters,".g2o")
    guessOutFilename = genFilename(dirNames[2]+"/guess_out", paramNames, parameters,".g2o")
    optFilename = genFilename(dirNames[3]+"/opt_out", paramNames, parameters,".g2o")
    figFilename = genFilename(dirNames[4]+"/res", paramNames, parameters,"")

    # get simulation data
    subprocess.call([binPath+"./g2o_simulator2d_noise", "-hasOdom", "-hasPointSensor",
                     "-nlandmarks", str(nlandmarks), "-simSteps", str(simSteps),
                     "-infoOdomPos", str(infoOdomPos), "-infoOdomAng", str(infoOdomAng),
                     "-infoPointSen", str(infoPointSen), simFilename])
    fixNode(simFilename, 1000+simSteps)
                     
    # get initial guess
    getInitialGuess(simFilename, guessInFilename)
    subprocess.call([binPath+"./g2o", "-i", "0", "-guessOdometry",
                     "-o", guessOutFilename, guessInFilename])

    # make g2o optimization
    subprocess.call([binPath+"./g2o", "-i", str(g2oIterations), "-guessOdometry",
                     "-i", "5",
                     #"-inc",
                     #"-robustKernel",
                     #"-robustKernelWidth",
                     #"-solver",
                     "-o", optFilename, guessInFilename])
                     
    # plot results
    plotResults(simFilename, guessOutFilename, optFilename, figFilename, [-15, 20], [-20, 20])
    
if __name__ == '__main__':
    main()

#imports
import subprocess
import sys
import time
import os
sys.path.append('../commons')
from slamFunctions import *


def main():
    
    # variables
    g2oIterations = 10
    
    # run g2o tests
    start_time = time.time()
    runG2O(g2oIterations)
    elapsed_time = time.time() - start_time
    print "Total time tests: " + str(elapsed_time) + " [s]"
    
def runG2O(g2oIterations):
    
    # paths
    dataPath = "data/Parque OHiggins/ohiggins.g2o"
    #dataPath = "data/Victoria Park Felipe/victoria.g2o"
    dataName = os.path.splitext(os.path.basename(dataPath))[0]
    guessOutPath = "res/guess_out_" + dataName + ".g2o"
    resPath = "res/opt_out_" + dataName + ".g2o"
    figPath = "res/res_" + dataName
    
    
    # get initial guess
    subprocess.call(["g2o", "-i", "0", "-guessOdometry",
                     "-o", guessOutPath, dataPath])
                     
    # plot results
    makeRealPlots(guessOutPath, figPath)
    #makeRealPlots(dataPath, figPath) 
     
if __name__ == '__main__':
    main()

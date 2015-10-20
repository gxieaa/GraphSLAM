#imports
import subprocess
import sys
import time
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
    dataPath = "data/Parque OHiggins/parqueOhiggins.g2o"
    #dataPath = "data/Victoria Park Felipe/victoriaPark.g2o"
    guessOutPath = "res/guess_out.g2o"
    resPath = "res/opt_out.g2o"
    figPath = "res/res"
    
    # get initial guess
    #subprocess.call(["g2o", "-i", "0", "-guessOdometry",
    #                 "-o", guessOutPath, dataPath])
                     
    # plot results
    #plotResults(guessOutPath, guessOutPath, guessOutPath, figPath, [-250, 150], [-200, 100])    
    #plotResults(dataPath, dataPath, dataPath, figPath, [-250, 150], [-200, 100])
    makeRealPlots (dataPath) 
     
if __name__ == '__main__':
    main()

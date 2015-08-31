# imports
import subprocess
import shlex
import matplotlib.pyplot as plt

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
    
    for i in range(n):
        runG2O(g2oIterations, nlandmarks, simSteps, infoOdomPos[i], infoOdomAng[i], infoPointSen[i])
    
def runG2O(g2oIterations, nlandmarks, simSteps, infoOdomPos, infoOdomAng, infoPointSen):

    binPath = "../../g2o-master/bin/"
    simFilename = genFilename("sim_out/sim_out", g2oIterations, simSteps, nlandmarks, 
                              infoOdomPos, infoOdomAng, infoPointSen,".g2o")
    guessInFilename = genFilename("guess_in/guess_in", g2oIterations, simSteps, nlandmarks, 
                                  infoOdomPos, infoOdomAng, infoPointSen,".g2o")
    guessOutFilename = genFilename("guess_out/guess_out", g2oIterations, simSteps, nlandmarks, 
                                    infoOdomPos, infoOdomAng, infoPointSen,".g2o")
    optFilename = genFilename("opt_out/opt_out", g2oIterations, simSteps, nlandmarks, 
                              infoOdomPos, infoOdomAng, infoPointSen,".g2o")
    figFilename = genFilename("figs/res", g2oIterations, simSteps, nlandmarks, 
                              infoOdomPos, infoOdomAng, infoPointSen,"")

    # get simulation data
    subprocess.call([binPath+"./g2o_simulator2d_noise", "-hasOdom", "-hasPointSensor",
                     "-nlandmarks", str(nlandmarks), "-simSteps", str(simSteps),
                     "-infoOdomPos", str(infoOdomPos), "-infoOdomAng", str(infoOdomAng),
                     "-infoPointSen", str(infoPointSen), simFilename])
    fSim = open(simFilename, 'a')
    fSim.write("FIX " + str(1000 + simSteps))
    fSim.close()
                     
    # get initial guess
    getInitialGuess(simFilename, guessInFilename)
    subprocess.call([binPath+"./g2o", "-i", "0", "-guessOdometry",
                     "-o", guessOutFilename, guessInFilename])

    # make g2o optimization
    subprocess.call([binPath+"./g2o", "-i", str(g2oIterations), "-guessOdometry",
                     #"-inc",
                     #"-robustKernel",
                     #"-robustKernelWidth",
                     #"-solver",
                     "-o", optFilename, guessInFilename])
                     
    # plot results
    plotResults(simFilename, guessOutFilename, optFilename, figFilename)

# functions
def genFilename(prefix, i, s, l, p, a, ps, sufix):
    return prefix + "_i_" + str(i) + "_s_" + str(s) + "_l_" + str(l) + "_p_" + str(p) +\
    "_a_" + str(a) + "_ps_" + str(ps) + sufix

def getInitialGuess(gtFilename, guessFilename):
    fGt = open(gtFilename, 'r')
    fGuess = open(guessFilename, 'w')

    for line in fGt:
        if line[0:4] == "EDGE" or line[0:3] == "FIX":
            fGuess.write(line)

    fGt.close()
    fGuess.close()

def plotResults(gtFilename, guessFilename, optFilename, figFilename):
    
    # get variables from file
    gtPoseX, gtPoseY, gtLandmarkX, gtLandmarkY = getData(gtFilename)
    guessPoseX, guessPoseY, guessLandmarkX, guessLandmarkY = getData(guessFilename)
    optPoseX, optPoseY, optLandmarkX, optLandmarkY = getData(optFilename)
    
    # create figure
    lw = 1
    f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
    # create initial guess plot
    gtLanPlt, = ax1.plot(gtLandmarkX, gtLandmarkY, '.', color = '#ff8e8e', linewidth = lw, label='GT landmarks')
    gtRobPlt, = ax1.plot(gtPoseX, gtPoseY, '.-', color = '#bbbbf9', linewidth = lw, label='GT robot path')
    lanPlt, = ax1.plot(guessLandmarkX, guessLandmarkY, 'r.', linewidth = lw, label='landmarks')
    robPlt, = ax1.plot(guessPoseX, guessPoseY, 'b.-', linewidth = lw, label='robot path')
    ax1.grid(True)
    #ax1.legend(handles=[robPlt, lanPlt, gtRobPlt, gtLanPlt], bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    ax1.set_title("Initial Guess")
    # create initial guess plot
    gtLanPlt, = ax2.plot(gtLandmarkX, gtLandmarkY, '.', color = '#ff8e8e', linewidth = lw, label='GT landmarks')
    gtRobPlt, = ax2.plot(gtPoseX, gtPoseY, '.-', color = '#bbbbf9', linewidth = lw, label='GT robot path')
    lanPlt, = ax2.plot(optLandmarkX, optLandmarkY, 'r.', linewidth = lw, label='landmarks')
    robPlt, = ax2.plot(optPoseX, optPoseY, 'b.-', linewidth = lw, label='robot path')
    ax2.grid(True)
    ax2.legend(handles=[robPlt, lanPlt, gtRobPlt, gtLanPlt], bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    ax2.set_title("After Solver")
    #plt.show()
    # print figure
    plt.xlim([-15, 20])
    plt.ylim([-20, 20])
    plt.savefig(figFilename + ".png", bbox_inches='tight')
    plt.savefig(figFilename + ".pdf", bbox_inches='tight')
    
def getData(dataFilename):
    
    # to fill arrays
    poseX = []
    poseY = []
    landmarkX = []
    landmarkY = []
    f = open(dataFilename)
    # get data loop
    for line in f:
        # split string
        lineWords = shlex.split(line)
        if lineWords[0] == "VERTEX_SE2":
            # get robot pose
            poseX.append(float(lineWords[2]))
            poseY.append(float(lineWords[3]))
        elif lineWords[0] == "VERTEX_XY":
            # get landmark position
            landmarkX.append(float(lineWords[2]))
            landmarkY.append(float(lineWords[3]))
    f.close()
    return poseX, poseY, landmarkX, landmarkY
    
if __name__ == '__main__':
    main()

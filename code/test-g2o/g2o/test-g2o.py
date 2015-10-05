# imports
import subprocess

def main():
    # variables
    binPath = "../../g2o-master/bin/"
    #dataPath = "../data/guess_in_i_10_s_300_l_300_p_5_a_50_ps_10.g2o"
    dataPath = "../data/simple_sim.g2o"
    resPath = "res.g2o"
    summaryPath = "sumarry.txt"
    gnudumpPath = "gnudump.txt"
    statsPath = "stats.txt"
    
    # run script
    subprocess.call([binPath+"./g2o", 
    "-i", "10", 
    "-guessOdometry",
    "-computeMarginals",
    #"-listRobustKernels", "-listSolvers", "-listTypes", 
    #"-v", "-printSolverProperties", 
    #"-summary", summaryPath, "-gnudump", gnudumpPath, "-stats", statsPath,
    #"-inc", 
    "-o", resPath, dataPath])

if __name__ == '__main__':
    main()
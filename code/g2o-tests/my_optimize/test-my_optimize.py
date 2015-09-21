# imports
import subprocess

def main():
    # variables
    buildPath = "../../g2o-master/build/"
    binPath = "../../g2o-master/bin/"
    dataPath = "../data/simple_corr2.g2o"
    #dataPath = "../data/simple_sim.g2o"
    #dataPath = "../data/simple_odom.g2o"
    #dataPath = "../data/guess_in_i_10_s_300_l_300_p_5_a_50_ps_10.g2o"
    #dataPath = "../data/guess_out_i_10_s_300_l_300_p_5_a_50_ps_10.g2o"
    resPath = "res.g2o"
    
    # compile
    subprocess.call(["make", "-C", buildPath]) 
    # run script
    subprocess.call([binPath+"./my_optimize", 
    "-i", "10", 
    "-o", resPath, dataPath])

if __name__ == '__main__':
    main()

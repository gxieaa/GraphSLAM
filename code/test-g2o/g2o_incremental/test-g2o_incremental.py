# imports
import subprocess

def main():
    # variables
    dataPath = "../data/guess_in_i_10_s_300_l_300_p_5_a_50_ps_10.g2o"
    #dataPath = "../data/guess_out_i_10_s_300_l_300_p_5_a_50_ps_10.g2o"
    #dataPath = "../data/guess_in_i_10_s_300_l_300_p_500_a_5000_ps_1000.g2o"
    #dataPath = "../data/incremental_corr.g2o"
    #dataPath = "../data/protocol.g2o"
    resPath = "res.g2o"
    
    # run script
    subprocess.call(["g2o_incremental", 
    "-i", dataPath,  
    #"-g", 
    "-v",
    #"-batch", "100"
    #"-update", "10"
    "-o", resPath])

if __name__ == '__main__':
    main()

# imports
import subprocess

def main():
    # variables
    dataPath = "../data/guess_out_i_10_s_300_l_300_p_5_a_50_ps_10.g2o"
    resPath = "res.g2o"
    
    # run script
    subprocess.call(["g2o_anonymize_observations",
    "-o", resPath, dataPath])

if __name__ == '__main__':
    main()

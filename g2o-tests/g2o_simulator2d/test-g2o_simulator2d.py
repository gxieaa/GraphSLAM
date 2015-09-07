# imports
import subprocess

def main():
    # variables
    binPath = "../../g2o-master/bin/"
    dataPath = "../data/guess_in_i_10_s_300_l_300_p_5_a_50_ps_10.g2o"
    resPath = "res.g2o"
    
    subprocess.call([binPath+"./g2o_simulator2d", 
    "-hasOdom", "-hasPointSensor",
    "-nlandmarks", "20", "-simSteps", "1",
    resPath])

if __name__ == '__main__':
    main()

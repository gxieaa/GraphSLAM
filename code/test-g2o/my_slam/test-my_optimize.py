# imports
import subprocess

def main():
    # variables
    binPath = "../../my-scripts/my-slam/build/"
    dataPath = "../data/simple_corr2_noise.g2o"
    resPath = "res.g2o"
    
    # compile
    #subprocess.call(["make", "-C", buildPath]) 
    # run script
    subprocess.call([binPath+"./my_slam", 
    "-i", "10",
    "-o", resPath, dataPath])

if __name__ == '__main__':
    main()

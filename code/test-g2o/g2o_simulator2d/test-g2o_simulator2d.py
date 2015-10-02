# imports
import time
import subprocess

def main():
    # variables
    binPath = "../../my-scripts/my-simulator/build/"
    #binPath = "../../g2o-master/bin/"
    resPath = "res.g2o"
    
    start_time = time.time()
    subprocess.call([binPath+"./my_simulator", 
    "-hasOdom", "-hasPointSensor",
    "-nlandmarks", "500",
    "-simSteps", "500",
    resPath])
    elapsed_time = time.time() - start_time
    print "Total time: " + str(elapsed_time) + " [s]"

if __name__ == '__main__':
    main()

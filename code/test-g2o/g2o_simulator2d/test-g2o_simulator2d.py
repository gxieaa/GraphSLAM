# imports
import time
import subprocess

def main():
    # variables
    resPath = "res.g2o"
    
    start_time = time.time()
    subprocess.call(["g2o_simulator2d", 
    "-hasOdom", "-hasPointSensor",
    "-nlandmarks", "500",
    "-simSteps", "500",
    resPath])
    elapsed_time = time.time() - start_time
    print "\nTotal time: " + str(elapsed_time) + " [s]"

if __name__ == '__main__':
    main()

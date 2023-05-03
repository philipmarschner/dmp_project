from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
import argparse
import sys




def main():
    """Main entry point allowing external calls
    
    Args:
      args ([str]): command line parameter list
    """

    print("running")

    dt = 1 / 500

    
    rtde_r = RTDEReceive("192.168.1.111")
    rtde_r.startFileRecording("dmp_collision_500hz_slaps3.csv")
    print("Data recording started, press [Ctrl-C] to end recording.")
    i = 0
    try:
        while True:
            t_start = rtde_r.initPeriod()
            start = time.time()
        
            sys.stdout.write("\r")
            sys.stdout.write("{:3d} samples.".format(i))
            sys.stdout.flush()
            rtde_r.waitPeriod(t_start)
            i += 1

    except KeyboardInterrupt:
        rtde_r.stopFileRecording()
        print("\nData recording stopped.")


if __name__== "__main__":

    main()
import motioncapture
import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("type")
    parser.add_argument("hostname")
    args = parser.parse_args()

    mc = motioncapture.connect(args.type, {"hostname": args.hostname})
    while True:
        mc.waitForNextFrame()
        for name, obj in mc.rigidBodies.items():
            print(name, obj.position, obj.rotation.z)

from dronekit import *
import sys
import time

class Solo():
    """docstring for ."""
    def __init__(self, virtual=False):
        self.vehicle = None
        self.connected = False

    def connect(self):
        # Connect to UDP endpoint (and wait for default attributes to accumulate)
        target = sys.argv[1] if len(sys.argv) >= 2 else 'udpin:0.0.0.0:14550'
        print('Connecting to ' + target + '...')
        self.vehicle = drone_connect(target, wait_ready=True)
        self.connected = True
        if not self.vehicle:
            print('bad.')


    def get_state(self):
        if not self.connected:
            print('vehicle not connected. Call .connect to connect.')
            return

        return self.vehicle.location.local_frame, self.vehicle.attitude

def main():
    vehicle = Solo()
    vehicle.connect()

    with open('solo_data.csv', 'w') as f:
        f.write('gps, imu\n')

    while True:
        with open('solo_data.csv', 'a') as f:
            f.write('{}, {}\n'.format(vehicle.get_state()))
        time.sleep(0.01)

    print("Done.")


if __name__ == "__main__":
    main()

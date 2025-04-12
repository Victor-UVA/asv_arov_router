from pymavlink import mavutil
import rclpy
from rclpy.node import Node
import threading
import time

class Accel_Listener(threading.Thread):
    def __init__(self, master):
        threading.Thread.__init__(self)
        self.master = master
        self.last_timestep = 0

    def run(self):
        while True:
            try:
                msg = self.master.messages['RAW_IMU']
            except:
                continue
            if msg != None and self.last_timestep != msg.to_dict()['time_usec']:
                print('Time ', msg.to_dict()['time_usec'], 'X Acc ', msg.to_dict()['xacc'])
                self.last_timestep = msg.to_dict()['time_usec']
            time.sleep(0.1)

class Accel_Y_Listener(threading.Thread):
    def __init__(self, master):
        threading.Thread.__init__(self)
        self.master = master
        self.last_timestep = 0

    def run(self):
        while True:
            try:
                msg = self.master.messages['RAW_IMU']
                if msg != None and self.last_timestep != msg.to_dict()['time_usec']:
                    print('Time ', msg.to_dict()['time_usec'], 'Y Acc ', msg.to_dict()['yacc'])
                    self.last_timestep = msg.to_dict()['time_usec']
            except:
                continue
            time.sleep(0.1)


class BlueROV_Listener(Node):
    def __init__(self):
        super().__init__('bluerov_listener')
    
        print('Hi from bluerov.')
        
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.master.wait_heartbeat()

        print("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))
        print('BlueROV connected!.')

        self.accel_listener = Accel_Listener(master=self.master)
        self.accel_listener.start()

        # self.accel_y_listener = Accel_Y_Listener(master=self.master)
        # self.accel_y_listener.start()

        # self.acc_timer = self.create_timer(0.01, self.get_acc)
        # self.accy_timer = self.create_timer(0.01, self.get_accy)

    def get_acc(self):
        msg = self.master.recv_match(type='RAW_IMU', blocking=False)
        if msg != None:
            print('X Acc ', msg.to_dict()['xacc'])

    def get_accy(self):
        msg = self.master.recv_match(type='RAW_IMU', blocking=False)
        if msg != None:
            print('Y Acc ', msg.to_dict()['yacc'])


def main():
    rclpy.init()

    bluerov_listener = BlueROV_Listener()

    rclpy.spin(bluerov_listener)

    bluerov_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

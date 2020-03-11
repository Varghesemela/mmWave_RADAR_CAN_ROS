import can
import struct
import select

class Radar(object):
    def __init__(self, can_channel='can1'):
        self.bus = can.Bus(interface='socketcan', channel=can_channel, fd=True)

    def spin(self):
        while True:
            try:
                print("try")
                can_data = self.bus.recv()
                print(can_data)
                #print(dir(can_data))
            except KeyboardInterrupt:
                break
            except select.error:
                pass

r = Radar()
r.spin()

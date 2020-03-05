
#!/usr/bin/python2
import can
import struct
import select
import rospy
from math import sqrt, atan2
from radar_msg.msg import *



class Radar_parse(object):
    Radar_IDs = {
    '0x11' : "Radar 1 header",
    '0x12' : "Radar 1 PCD   ",
    '0x13' : "Radar 1 Noise profile ",
    '0x21' : "Radar 2 header",
    '0x22' : "Radar 2 PCD   ",
    '0x23' : "Radar 2 Noise profile ",
    }

    def __init__(self):
        self.radar1_msg = RadarScan()
        self.radar2_msg = RadarScan()        
        self.radar1_points = RadarPoints()
        self.topic_name = "/radar1/data"
        self.pub1_radar = rospy.Publisher(self.topic_name, RadarPoints, queue_size=10)
        self.radar2_points = RadarPoints()
        self.topic_name = "/radar2/data"
        self.pub2_radar = rospy.Publisher(self.topic_name, RadarPoints, queue_size=10)
        self.Num_of_objects = 0
        self.limit = 0

    def __delete__(self, instance):
        print "deleted in descriptor object"
        del self.radar_msg
    
    def parse(self, radar_data):
        message_ID = hex(radar_data.arbitration_id)
        if(message_ID in Radar_parse.Radar_IDs):
            if(message_ID == '0x11'):
                self.Num_of_objects = struct.unpack('B',radar_data.data[28:29])
                self.Num_of_objects = self.Num_of_objects[-1]
                print(self.Num_of_objects)
                self.limit = 0
            if(message_ID == '0x12'):
                    PCDX = []                       
                    if((self.Num_of_objects-self.limit)/4 > 0):
                        framelen = 4
                    else:
                        framelen = self.Num_of_objects - self.limit   
                    #for i in range(len(radar_data.data)/4):
                    #    PCDX_temp = (radar_data.data[(4*i)+0:(i*4)+4])
                    #    PCDX_temp = PCDX_temp[::-1]
                    #    tmp = struct.unpack('>f', PCDX_temp)
                    #    PCDX.append(tmp[-1])
                    #print(PCDX)
                    count = (len(radar_data.data)/4)
                    PCDX_temp = radar_data.data[::-1]
                    PCDX = struct.unpack('>{}f'.format(count), PCDX_temp)
                    
                    
                    for i in range(framelen):  
                        self.radar1_msg.point_id = self.limit
                        self.radar1_msg.y = PCDX[(4*i)+3]*-1
                        self.radar1_msg.x = PCDX[(4*i)+2]
                        self.radar1_msg.z = PCDX[(4*i)+1]
                        self.radar1_msg.velocity = PCDX[(4*i)+0]
                        self.radar1_msg.range = float(sqrt(PCDX[(4*i)+2]**2+PCDX[(4*i)+ 3]**2)) 
                        self.radar1_msg.azimuth = float(atan2(PCDX[(4*i)+ 3], PCDX[(4*i)+ 2]))
                
                        self.radar1_points.radarscan.append(self.radar1_msg)
                        del self.radar1_msg                        
                        self.radar1_msg = RadarScan()  
                        
                        print(self.radar_points.radarscan)                              
                        if(self.limit <= self.Num_of_objects):
                            self.limit+= 1
                            #print("hello")
                    if(self.limit == self.Num_of_objects):
                            #print(self.limit) 
                            self.pub1_radar.publish(self.radar1_points)
                            self.radar1_points.radarscan = []
                            self.limit = 0
                            self.Num_of_objects = 0   
            
            if(message_ID == '0x21'):
                self.Num_of_objects = struct.unpack('B',radar_data.data[28:29])
                self.Num_of_objects = self.Num_of_objects[-1] 
                print(self.Num_of_objects)
                self.limit = 0
            if(message_ID == '0x22'):
                    PCDX = []                       
                    if((self.Num_of_objects-self.limit)/4 > 0):
                        framelen = 4
                    else:
                        framelen = self.Num_of_objects - self.limit   
                    #for i in range(len(radar_data.data)/4):
                    #    PCDX_temp = (radar_data.data[(4*i)+0:(i*4)+4])
                    #    PCDX_temp = PCDX_temp[::-1]
                    #    tmp = struct.unpack('>f', PCDX_temp)
                    #    PCDX.append(tmp[-1])
                    #print(PCDX)
                    count = (len(radar_data.data)/4)
                    PCDX_temp = radar_data.data[::-1]
                    PCDX = struct.unpack('>{}f'.format(count), PCDX_temp)
                    
                    
                    for i in range(framelen):  
                        self.radar2_msg.point_id = self.limit
                        self.radar2_msg.y = PCDX[(4*i)+3]*-1
                        self.radar2_msg.x = PCDX[(4*i)+2]
                        self.radar2_msg.z = PCDX[(4*i)+1]
                        self.radar2_msg.velocity = PCDX[(4*i)+0]
                        self.radar2_msg.range = float(sqrt(PCDX[(4*i)+2]**2+PCDX[(4*i)+ 3]**2)) 
                        self.radar2_msg.azimuth = float(atan2(PCDX[(4*i)+ 3], PCDX[(4*i)+ 2]))
                
                        self.radar2_points.radarscan.append(self.radar2_msg)
                        del self.radar2_msg                        
                        self.radar2_msg = RadarScan()  
                        
                        print(self.radar2_points.radarscan)                              
                        if(self.limit <= self.Num_of_objects):
                            self.limit+= 1
                            #print("hello")
                    if(self.limit == self.Num_of_objects):
                            #print(self.limit) 
                            self.pub2_radar.publish(self.radar2_points)
                            self.radar2_points.radarscan = []
                            self.limit = 0
                            self.Num_of_objects = 0            
 
            
class Radar(object):
    def __init__(self, can_channel='can1'):
        self.bus = can.Bus(interface='socketcan', channel=can_channel, fd=True)
        self.Radar_parse = Radar_parse()

    def spin(self):
        while True:
            radar_data = self.bus.recv()
                #print(radar_data)
            self.Radar_parse.parse(radar_data)


if __name__ == '__main__':
    rospy.init_node('RADAR_can')
    driver = Radar('can1')
    driver.spin()

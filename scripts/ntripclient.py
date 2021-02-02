#!/usr/bin/python

import rospy
import socket
import re
import binascii

#from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message
from mavros_msgs.msg import RTCM

from httplib import HTTPConnection
from base64 import b64encode
from threading import Thread
from datetime import *
from time import sleep

class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    def to_dec_minutes(self, degree):
        dd = abs(degree)
        minutes,seconds = divmod(dd*3600,60)
        degrees,minutes = divmod(minutes,60)
        return str(int(degrees)) + "{:0>2d}".format(int(minutes)) + "{:8.7f}".format(seconds/60)

    def lat_dir(self, latitude):
        if latitude > 0:
            return 'N'
        else:
            return 'S'

    def long_dir(self, longitude):
        if longitude > 0:
            return 'E'
        else:
            return 'W'

    def checksum(self, sentence):

        """ Remove any newlines """

        calc_cksum = 0
        for s in sentence:
            calc_cksum ^= ord(s)

        """ Return the nmeadata, the checksum from
            sentence, and the calculated checksum
        """
        return sentence + "*" + hex(calc_cksum)[2:]

    def run(self):
        dt = datetime.utcnow()

        rospy.loginfo(str(dt))
        # create gga
        #"$GPGGA,%02.0f%02.0f%05.2f,%012.7f,%s,%013.7f,%s,1,10,1.2,%.4f,M,-2.860,M,,0000",
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close'
        }

        while not self.stop:
            try:
                gga = "$GPGGA,{:0>2d}".format(dt.hour) + "{:0>2d}".format(dt.minute) + "{:0>2d}".format(dt.second)+ ".{:0>2d}".format(int(dt.microsecond / 10000))
                gga += ',' + self.to_dec_minutes(self.ntc.latitude) + ',' + self.lat_dir(self.ntc.latitude)
                gga += ',' + self.to_dec_minutes(self.ntc.longitude) + ',' + self.long_dir(self.ntc.longitude)
                gga += ',1,10,1.2,{:.4f},M,-2.860,M,,0000'.format(self.ntc.altitude)
                gga = self.checksum(gga)
                rospy.loginfo(gga)
                connection = HTTPConnection(self.ntc.ntrip_server + ':' + str(2101))
                connection.request('GET', '/'+self.ntc.ntrip_stream, gga, headers)

                response = connection.getresponse()
                if response.status != 200: raise Exception(response.status)

                rmsg = RTCM()
                while not self.stop:
                    data = response.read(1)
                    if data!=chr(211):
                        continue
                    l1 = ord(response.read(1))
                    l2 = ord(response.read(1))
                    pkt_len = ((l1&0x3)<<8)+l2

                    pkt = response.read(pkt_len)
                    parity = response.read(3)
                    if len(pkt) != pkt_len:
                        rospy.logerr("Length error: {} {}".format(len(pkt), pkt_len))
                        continue
                    rmsg.header.seq += 1
                    rmsg.header.stamp = rospy.get_rostime()
                    rmsg.data = data + chr(l1) + chr(l2) + pkt + parity
                    self.ntc.pub.publish(rmsg)

                connection.close()

            except socket.timeout:
                sleep(0.01)
                pass  # we'll ignore timeout errors and reconnect
            except Exception as e:
                print("Request exception `{}`, exiting".format(e))
                break


class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic', '/mavros/gps_rtk/send_rtcm')
        self.ntrip_server = rospy.get_param('~ntrip_server', 'rtk2go.com')
        self.ntrip_stream = rospy.get_param('~ntrip_stream', 'HoustonMech')
        self.latitude = rospy.get_param('~latitude', 29.549109)
        self.longitude = rospy.get_param('~longitude', -95.049529)
        self.altitude = rospy.get_param('~altitude', 10)

        self.pub = rospy.Publisher(self.rtcm_topic, RTCM, queue_size=10)

        self.connection = None
        self.connection = ntripconnect(self)
        self.connection.start()

    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True

if __name__ == '__main__':
    c = ntripclient()
    c.run()


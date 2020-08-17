'''
Create NMEA strings.
'''

# import socket
import fire
import json
import random
import logging
import time
# from core import NMEASender

class NMEAStrings(object):

    def __init__(self):
        self.log = logging.getLogger("NMEAStr")
        self.log.info(f'control: Initialized NMEAStrings')
        self.oldchecksum = "poopy"

    def raw(self, nmea_string):
        """ Send raw NMEA string without any validation """
        # self.nmea_sender.send(nmea_string)
        return(nmea_string)

    def ROR(self, starboard, port):
        """ Send an Rudder Order Status NMEA demand to the starboard and port rudders """
        nmea_cmd = f"$PROR,{starboard},A,{port},A"

        checksum = self.NMEA_CRC(nmea_cmd)
        nmea_cmd_chksum = f"{nmea_cmd}*{checksum}"

        return(nmea_cmd_chksum)

    def RSA(self, starboard, port):
        """ Send an Rudder Order Status NMEA demand to the starboard and port rudders """
        nmea_cmd = f"$AIRSA,{starboard},A,{port},A"
        self.log.info("RSA ---- %s" % nmea_cmd)
        checksum = self.NMEA_CRC(nmea_cmd)
        nmea_cmd_chksum = f"{nmea_cmd}*{checksum}"
        return(nmea_cmd_chksum)



        # if checksum == self.oldchecksum:
        #     return(False)
        # else: 
        #     self.oldchecksum = checksum
        #     return(nmea_cmd_chksum)
  

    def TRC(self, index, rpm=0, pitch=0, azimuth=0):
        """ Send a Thruster Control Data NMEA command inidicating what RPM, pitch and azimuth is demanded from which thruster (defined by index) """
        nmea_cmd = f"$PTRC,{index},{rpm},P,{pitch},P,{azimuth},B"
        # self.nmea_sender.send(nmea_cmd)
        checksum = self.NMEA_CRC(nmea_cmd)
        nmea_cmd_chksum = f"{nmea_cmd}*{checksum}"
        return(nmea_cmd_chksum)

        # if checksum == self.oldchecksum:
        #     return(False)
        # else: 
        #     self.oldchecksum = checksum
        #     return(nmea_cmd_chksum)
        
    def WPT(self, lat=0, lng=0, wp_number=0, starting_pnt=0 , path_length=0, valid=0):
        nmea_cmd = f"$CAWPT,{lat},{lng},{wp_number},{starting_pnt},{path_length},{valid},B"
        return(nmea_cmd)
    
    def WPT_ARRAY(self, array_lat, array_lng):
        # lat_array2list = array_lat.tolist()
        # lng_array2list = array_lng.tolist()
        lat = "|".join(map(str,array_lat)) # lat_array2list))
        lng = "|".join(map(str,array_lng)) # lng_array2list))
        nmea_cmd = f"$CAWPTA,{lat},{lng}"
        checksum = self.NMEA_CRC(nmea_cmd)
        nmea_cmd_chksum = f"{nmea_cmd}*{checksum}"

        # if checksum == self.oldchecksum:
        #     return(False)
        # else: 
        #     self.oldchecksum = checksum
        return(nmea_cmd_chksum)
    
    def PLY(self, mmsi = 0, time = 0, X1 = 0, X2 = 0, X3 = 0, Y1 = 0, Y2 = 0, Y3 = 0, H1 = 0):
        # nmea_cmd = f"$CAPLY,{int(mmsi)},{time},{X1}|{X2}|{X3},{Y1}|{Y2}|{Y3},B"
        nmea_cmd = f"$CAPLY,{int(mmsi)},{time},{X1}|{X2}|{X3},{Y1}|{Y2}|{Y3},{H1}"
        checksum = self.NMEA_CRC(nmea_cmd)
        nmea_cmd_chksum = f"{nmea_cmd}*{checksum}"
        
        if checksum == self.oldchecksum:
            return(False)
        else: 
            self.oldchecksum = checksum
            return(nmea_cmd_chksum)
        # self.log.info(nmea_cmd)
        # self.nmea_sender.send(nmea_cmd_chksum)
        return(nmea_cmd_chksum)

    def CAM(self, theta, cam):
        t = time.time()
        # sentence = f'$CACAM,{t},{cam_nr},{Cw[0]},{Cw[1]},{Cw[2]},{fov_left},{fov_right},{conf_scores},{angles},{left_angles},{right_angles},{distances},{pose_heads}'
        
        left_angles = theta -10 + random.randint(0, 5)
        right_angles =  left_angles + 20

        nmea_cmd = f'$CACAM, {t}, {cam} ,0.0, 0.85, -2.8, 67.3534310408989, -0.6218476031895399, 0.9694215059280396|0.8694215059280396 , 10|45,{left_angles},{right_angles}, 40|10, 180|45'
        checksum = self.NMEA_CRC(nmea_cmd)
        nmea_cmd_chksum = f"{nmea_cmd}*{checksum}"
        
        if checksum == self.oldchecksum:
            return(False)
        else: 
            self.oldchecksum = checksum
            return(nmea_cmd_chksum)
        # self.log.info(nmea_cmd)
        # self.nmea_sender.send(nmea_cmd_chksum)
        return(nmea_cmd_chksum)

    def GPS_GGA(self, lat, lon):
        #GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        #$GPGGA,093342.84,5153.3414,N,00425.0933,E,2,15,0.7,-2.4,M,48.0,M,,*42
        lat = lat*100.0
        lon = lon*100.0
        nmea_cmd = f'$GPGGA,123519,{lat},N,{lon},E,2,15,0.7,-2.4,M,48.0,M,,'
        checksum = self.NMEA_CRC(nmea_cmd)
        nmea_cmd_chksum = f"{nmea_cmd}*{checksum}"

        nmea_cmd_chksum = f"$GPGGA,093342.84,5153.3414,N,00425.0933,E,2,15,0.7,-2.4,M,48.0,M,,*42"
        
        if checksum == self.oldchecksum:
            return(False)
        else: 
            self.oldchecksum = checksum

        return(nmea_cmd_chksum)


    def CAGPS(self, mmsi, lat, lon):
        lat = lat*100.0
        lon = lon*100.0
        nmea_cmd = f'$CAGPS,{mmsi},{lat},{lon}'
        checksum = self.NMEA_CRC(nmea_cmd)
        nmea_cmd_chksum = f"{nmea_cmd}*{checksum}"
        
        if checksum == self.oldchecksum:
            return(False)
        else: 
            self.oldchecksum = checksum
        
        return(nmea_cmd_chksum)


    def NMEA_HDT(self, heading):
        nmea_cmd_chksum = f'HEHDT,92.42,T*22'
        return(nmea_cmd_chksum)

    # def SHP(self, p1_lat, p1_lng, p2_lat, p2_lng, p3_lat, p3_lng, p4_lat, p4_lng, mmsi)

    def reset(self):
        """ Convenience method to reset most types of boats (up to three thrusters) by sending zero commands for ROR and TRC """
        self.ROR(0,0)
        self.TRC(0,0,0,0)
        self.TRC(1,0,0,0)
        self.TRC(2,0,0,0)

    def NMEA_CRC(self, InString):
        #-- We don`t use the $ in the checksum
        mycopy=InString[InString.find("$")+1:] # 150] #99]
        #-- Get the ASC of the first Char
        crc = ord(mycopy[0:1])
        #-- Use a loop to Xor through the string
        for n in range(1,len(mycopy)): #-1):
            crc = crc ^ ord(mycopy[n:n+1])

            #-- Pass the data back as a HEX string
        return '%X' % crc

# if __name__ == '__main__':
#     fire.Fire(NMEAControls)
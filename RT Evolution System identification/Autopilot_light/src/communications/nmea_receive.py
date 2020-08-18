import ais
import pynmea2
import numpy as np
import logging
import src.logger as logger
import sys
import time
import json
import struct

# Todo add speed perameter
try:
    from src import nav_mat
    from src import nav_tools as nt
except Exception as e:
    import nav_mat
    import nav_tools as nt

class UDPProcessor():
    def __init__(self, settings):
        
        self.cambpix = settings['MODI']['cambpix']
        self.message_types = settings['NMEA']['message_types']
        self.my_mmsi = settings['GENERAL']['my_mmsi']

        self.ship_update = nav_mat.clear_mat(settings['SHIP_MAT']['ship_mat_parameters'])
        self.delete_mat = self.ship_update
        self.rmc_speed = False

        self.logging = False
        # Initiate logger

        if self.logging == True:
            logger.worker_configurer(settings['LOGGER']['log_q'])
            self.log = logging.getLogger('UDPProcessor')
            self.log.info("started")


        self.ROI_ships = settings['SHIP_MAT']['ROI_ships']  # This is the distance to which boats should be considered


    def process_NMEA(self, nmea_message, for_send, ships_mat):
        # ToDo: Make each one into a function
        # self.og.inf(f"{ships_mat[0, 0, 1]}, {ships_mat[0, 0, 2]}, {ships_mat[0, 0, 3]}")
        # Types = $XSVEL , $PRDID, $HCMTW, $PTCF, $GPZDA, $PRDID, "$HCMTW", "SDVHW", "IIMWV"
        #       "$PTCF", "$GPZDA", "$PRDID", "$GPRMC", "$HCHDG", "$GPGLL", "GPVTG"
        s = nmea_message
        p = s.split(',')[0]
        # print(p)
        # self.log.info(f"nmea_revieve nmea : lat {ships_mat[0, 0, 2]}")
        if p in self.message_types:  # Inor types that are not defined in the config file
            if p == "$GPGLL":
                try:
                    ships_mat[:, 0, 1] = self.my_mmsi

                    nmea = pynmea2.parse(s)

                    ships_mat[0, 0, 2] = float(nmea.latitude)
                    ships_mat[0, 0, 3] = float(nmea.longitude)
                    # ? no speed in any GLL ?
                    # ships_mat[0, 0, 4] = nmea.spd_over_grnd
                    ships_mat[0, 0, 0] = nt.get_time()

                except Exception as e:
                    if self.logging == True:
                        self.log.error(f"nmea_receive : Except - GPGLL - {e}")
                    else:
                        pass
                return (for_send, ships_mat)

            if p == '$GPRMC':
                nmea = pynmea2.parse(s)
                try:
                    ships_mat[:, 0, 1] = self.my_mmsi
                    ships_mat[0, 0, 2] = nmea.latitude
                    ships_mat[0, 0, 3] = nmea.longitude
                    ships_mat[0, 0, 5] = nmea.spd_over_grnd * 1.943846
                    ships_mat[0, 0, 7] = nmea.true_course

                    self.rmc_speed = True  # Can use external speed


                except Exception as e:
                    if self.logging == True:
                        self.log.error(f"nmea_receive : Except - GPRMC - {e}")
                    else:
                        pass

                return (for_send, ships_mat)


            if p == '$GORPM':
                nmea = pynmea2.parse(s)

                try:

                    ships_mat[:, 0, 1] = self.my_mmsi
                    ships_mat[2, 0, 2] = nmea.data[2]
                    # ships_mat[2, 0, 3] = nmea.longitude
                    # ships_mat[2, 0, 5] = nmea.spd_over_grnd * 1.943846
                    # ships_mat[2, 0, 7] = nmea.true_course
                    #                   # nmea.rpm
                    #
                    # self.rmc_speed = True  # Can use external speed


                except Exception as e:
                    if self.logging == True:
                        self.log.error(f"nmea_receive : Except - GORPM - {e}")
                    else:
                        pass

                return (for_send, ships_mat)

            if p == '$GOTRD':
                # nmea = pynmea2.parse(s)
                custom = s.split(',')
                # print(custom)
                try:
                    ships_mat[:, 0, 1] = self.my_mmsi
                    if custom[1]=='0':
                        ships_mat[2,:,6] = custom[-1][:-5]
                        ships_mat[2,:,9] = float(custom[2])*266/100
                    if custom[1]=='1':
                        ships_mat[2,:,7] = custom[-1][:-5]
                        ships_mat[2, :, 10] = float(custom[2])*266/100
                    if custom[1]=='2':
                        ships_mat[2,:,8] = custom[-1][:-5]
                        ships_mat[2, :, 11] = float(custom[2])*266/100

                except Exception as e:
                    if self.logging == True:
                        self.log.error(f"nmea_receive : Except - GORPM - {e}")
                    else:
                        pass

                return (for_send, ships_mat)

            if p == '$GOROR':
                poo =s.split(",")
                # print(1, poo)
                # nmea = pynmea2.parse(s)
                try:

                    ships_mat[:, 0, 1] = self.my_mmsi
                    # ships_mat[2, 0, 3] = poo[1]
                    # ships_mat[2, 0, 3] = nmea.longitude
            # ships_mat[2, 0, 5] = nmea.spd_over_grnd * 1.943846
                    # ships_mat[2, 0, 7] = nmea.true_course
                    #                   # nmea.rpm
                    #
                    # self.rmc_speed = True  # Can use external speed


                except Exception as e:
                    if self.logging == True:
                        self.log.error(f"nmea_receive : Except - GORSA - {e}")
                    else:
                        pass

                return (for_send, ships_mat)
            if p == '$GORSA':
                poo =s.split(",")
                # print(poo[3])
                # print(poo)
                # print(1, p
                #
                # oo)
                # nmea = pynmea2.parse(s)
                try:

                    ships_mat[:, 0, 1] = self.my_mmsi
                    ships_mat[2, 0, 3] = poo[3]
                    # ships_mat[2, 0, 3] = nmea.longitude
            # ships_mat[2, 0, 5] = nmea.spd_over_grnd * 1.943846
                    # ships_mat[2, 0, 7] = nmea.true_course
                    #                   # nmea.rpm
                    #
                    # self.rmc_speed = True  # Can use external speed


                except Exception as e:
                    if self.logging == True:
                        self.log.error(f"nmea_receive : Except - GORSA - {e}")
                    else:
                        pass

                return (for_send, ships_mat)

            if p == '$GPGGA':

                # try:
                ships_mat[0, 0, 0] = time.time()  # This time supports microseconds
                nmea = pynmea2.parse(s)
                ships_mat[:, 0, 1] = self.my_mmsi

                if nmea.latitude > 1 and nmea.longitude > 1:

                    # Calculate speed if not available
                    if self.rmc_speed == False:
                        # quick and dirty speed calculation
                        global_distance = nt.distance(ships_mat[0, 0, 2], ships_mat[0, 0, 3], nmea.latitude,
                                                      nmea.longitude)
                        # delta_latitude  = np.absolute(ships_mat[0, 0, 2] - nmea.latitude)   #Calculate the change in position for speed calc
                        # delta_longitude = np.absolute(ships_mat[0, 0, 3] - nmea.longitude)  #Calculate the change in position for speed calc
                        if self.logging == True:
                            self.log.info(f"Got a GPGGA rmc is {self.rmc_speed} distace {global_distance} ")

                        delta_time = time.time() - ships_mat[0, 0, 0]
                        if global_distance > 0:
                            ships_mat[0, 0, 5] = np.divide(global_distance,
                                                           delta_time) * 1000  # Low Precision Speed Calc
                            if self.logging == True:
                                self.log.info(f"nmea_recieve ln 97: calculated speed is {ships_mat[0, 0, 5]}")

                    # Update old values of lat and long after calculation of speed
                    ships_mat[0, 0, 2] = nmea.latitude
                    ships_mat[0, 0, 3] = nmea.longitude

                # except Exception as e:
                # 	self.log.info(f"nmea_receive : Except - GPGGA - {e}")

                return (for_send, ships_mat)

            if p == '$HCHDT':
                try:
                    nmea = pynmea2.parse(s)
                    ships_mat[0, 0, 4] = nmea.heading

                except Exception as e:
                    if self.logging == True:
                        self.log.error(f"nmea_receive : Except - $HCHDG - {e}")
                    else:
                        pass

                return (for_send, ships_mat)

            if p == '$HCHDG':
                try:
                    nmea = pynmea2.parse(s)
                    ships_mat[0, 0, 4] = nmea.heading
                    if self.logging == True:
                        self.log.info(f"nmea_recieve : HCHDG heading {ships_mat[0, 0, 4]}")
                    else:
                        pass

                except Exception as e:
                    self.log.error(f"nmea_receive : Except - $HCHDG - {e}")

                return (for_send, ships_mat)

            if p == '$HEHDT':
                try:
                    nmea = pynmea2.parse(s)
                    # print(nmea)
                    ships_mat[0, 0, 4] = nmea.heading
                # self.log.info(f"nmea_recieve HEHDT: heading {ships_mat[0, 0, 4]}")

                except Exception as e:
                    if self.logging == True:
                        self.log.debug(f"nmea_receive : Except - HEHDT - {e}")
                    else:
                        pass
                return (for_send, ships_mat)
            
            # $SDDBT,11.2518,f,3.4295,M,1.8753,F*09
            # $SDDPT,7.28251,0.0933068,10000*46
            if p == '$SDDBT':
                # if self.logging == True:
                #     self.log.debug(f"We got an SDDBT") 
                # else:
                #     print(f"We got an SDDBT")
                try:
                    sonar = pynmea2.parse(s) #UKC
                    ships_mat[0, 0, 6] = sonar.depth_meters
                    if self.logging == True:
                        self.log.debug(f"nmea_recieve : this is the sonar {sonar.depth_meters}")

                
                # self.log.info(f"nmea_recieve HEHDT: heading {ships_mat[0, 0, 4]}")

                except Exception as e:
                    if self.logging == True:
                        self.log.debug(f"nmea_receive : Except - SDDBT - {e}")
                    else: 
                        pass
                return (for_send, ships_mat)
            
            if p == '$SDDPT':
                # if self.logging == True:
                #     self.log.debug(f"We got an SDDBT") 
                # else:
                #     print(f"We got an SDDBT")
                try:
                    sonar = pynmea2.parse(s) #UKC
                    ships_mat[0, 0, 6] = sonar.depth
                # self.log.info(f"nmea_recieve HEHDT: heading {ships_mat[0, 0, 4]}")

                except Exception as e:
                    if self.logging == True:
                        self.log.debug(f"nmea_receive : Except - SDDPT - {e}")
                    else:
                        pass


        else:
            return (for_send, ships_mat)

    def process_RAIS(self, ais_message, for_send, ships_mat):
        # I removed the printing of error for troubleshooting a different part of the code
        # The Error messages are as follows from the cambridge pixel
        # AIS_1 exception Ais5: AIS_ERR_BAD_BIT_COUNT
        # nmea_receive AIS_0 exception ais.decode: unknown message - Q
        # AIS_1 exception ais.decode: unknown message - 0

        s = ais_message

        ais_message = s.split(',')  # [5]

        try:
            p = ais_message[5]
            q = ais.decode(p, 0)

        except Exception:
            # self.log.error(f'nmea_receive AIS_0 exception {e}')
            # self.log.error(p)

            try:
                q = ais.decode(p, 2)
            # self.log.info("process_RAIS Success! 1")
            # self.log.info(q)
            except Exception as e2:
                # self.log.error(f' AIS_1 exception {s}')
                # self.log.error(f' AIS_1 exception {e2}')
                return (for_send, ships_mat)

        # self.log.info(q)

        mmsi = float(q['mmsi'])

        if mmsi == self.my_mmsi:  # ths is is our ship

            return (for_send, ships_mat)

        if 'heading' in q:
            heading_ais = q['heading']

        elif 'cog' in q:
            heading_ais = q['cog']

        else:
            heading_ais = 0

        if 'sog' in q:
            sog = q['sog']

        else:
            sog = 0

        if 'y' in q:
            self.lat_2 = q['y']
        # self.log.info(f"nmea_recieve  AIS : {self.lat_2}")

        else:
            return (for_send, ships_mat)

        if 'x' in q:
            self.lng_2 = q['x']

        else:
            return (for_send, ships_mat)

        self.ship_update[0] = nt.get_time()  #
        self.ship_update[1] = mmsi
        self.ship_update[2] = self.lat_2
        self.ship_update[3] = self.lng_2
        self.ship_update[4] = heading_ais
        self.ship_update[5] = sog

        # self.log.info(f"nmea_receive self.ship_update = {self.ship_update}")
        if self.lng_2 > ships_mat[0, 0, 3] - self.ROI_ships and self.lng_2 < ships_mat[
            0, 0, 3] + self.ROI_ships and self.lat_2 > ships_mat[0, 0, 2] - self.ROI_ships and self.lat_2 < ships_mat[
            0, 0, 2] + self.ROI_ships:
            ships_mat = nav_mat.ship_update(ships_mat, self.ship_update)
        # else:
        # self.log.info("nmea_receive: No nearby ships detected")

        # self.og.info(ships_mat)

        return (for_send, ships_mat)

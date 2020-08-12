"""
The sender is used for sending to different targets from different threades
"""
import serial 
import time

from threading import Thread

class UDPSender():
    def __init__(self, ip, port):
        try:
            from src.communications.udp_manager import NMEASender
        except ImportError as e:
            print(e)
            from communications.udp_manager import NMEASender

        self.nmea_send = NMEASender(ip, port, False)
        print(f'Sender: Started sender')
        self.old_recieved = '*00'

    def send_string(self, recieved, for_send, ships_mat):
        if for_send is not False:
            # print(f"sender for_send : {for_send}")
            self.nmea_send.send(for_send)

    def send_string_main(self, recieved, for_send, ships_mat):
        if for_send is not False:
            # print(f"sender for_send : {for_send}")
            self.nmea_send.send(for_send)
        return(for_send, ships_mat)
        # if for_send != False: 
        #     chksum = for_send[-2:]
        #     if chksum != self.old_recieved:
        #         self.nmea_send.send(for_send)
        #         self.old_recieved = chksum

class SerialSender():
    def __init__(self, port='/dev/ttyACM0', baudrate=500000):
        #import Serial
        self.port = port
        self.baudrate = baudrate
        # self.ser = serial.Serial(port, baudrate, timeout=10000)
        # self.ser = serial.Serial(port, baudrate, write_timeout = None, rtscts = False, dsrdtr = False)
        self.ser = serial.Serial(port, baudrate, write_timeout = None, timeout = 0)
        time.sleep(2)
        
        # self.ser.flushInput()  # Flush startup text in serial input
        self.old_recieved = '*00'

        self.read_loop()

    def send_string(self, recieved, for_send, ships_mat):

        # if for_send != False: 
        # self.ser = serial.Serial(self.port, self.baudrate, write_timeout = None)
        # print(f'this is sender for send {for_send}')
        # sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))
        # print(f'{for_send}\n')
        self.ser.write(str.encode(for_send + '\n')) # Send g-code block
        self.ser.flushOutput()

    def read_loop(self):    
        def __read():
            while True:
                # Debug stuff
                r = self.ser.readline()
                # print("Serial response: ", r)
                # time.sleep(0.001)    

        t = Thread(target=__read)
        t.start()

    # def send_start()

    def send_close(self):
        f.close()
        s.close()


def testme():
    SendArduino = SerialSender('/dev/ttyACM0', 115200)
    received = None
    ships_mat = None
    import time

    timer = 0.9

    SendArduino.send_string(received, "$PROR,6,A,7,A*1E", ships_mat)
    time.sleep(timer)
    

    SendArduino.send_string(received, "$PROR,33,A,34,A*18", ships_mat)
    time.sleep(timer)

class SerialDummy():
    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        #import Serial
        print("Serial is not being used")

        # self.ser = serial.Serial(port, baudrate, timeout=0.5)
        # # self.ser.flushInput()  # Flush startup text in serial input
        # self.old_recieved = '*00'

    def send_string(self, recieved, for_send, ships_mat):
        a = "b"

        # if for_send != False: 
        # print(f'This would be sent {for_send}')
        # sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))
        # print(f'{for_send}\n')
        # self.ser.write(str.encode(for_send + '\n')) # Send g-code block
        # self.ser.flushOutput()

    # def send_start()

    def send_close():
        print("SerialDummy: closed")



def testme():
    SendArduino = SerialSender('/dev/ttyACM0', 115200)
    received = None
    ships_mat = None
    import time
    timer = 0.9

    SendArduino.send_string(received, "$PROR,6,A,7,A*1E", ships_mat)
    time.sleep(timer)
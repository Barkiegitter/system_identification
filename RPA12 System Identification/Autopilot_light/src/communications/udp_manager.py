import socket
import threading
import time

import pynmea2


'''
A duplex NMEA communication client communicating with a NMEA server (simulator, ship) over UDP.
It has a NMEA sender and receiver component.
'''

class NMEAClient(object):

    def __init__(self, server_ip, recv_port, send_port, log_dir=None):
        local_ip = 'i dont know why this'
        self.server_ip = server_ip
        self.recv_port = recv_port
        self.send_port = send_port

        self.sender = NMEASender(server_ip, send_port)
        self.receiver = NMEAReceiver(local_ip)

        def callback(nmea_string):
            pass
            # print("CB", nmea_string)

        self.receiver.listen(callback)


class NMEASender:

    def __init__(self, ip, port, verbosity=False):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.verbosity = verbosity

        if verbosity:
            print(f"NMEASender -- {ip}:{port}")

    def send(self, nmea_sentences):
        if type(nmea_sentences) is not list: #TODO: Check for other iterable types, is also fine
            nmea_sentences = [nmea_sentences]
        try:
            nmea_sentences = [str(n) for n in nmea_sentences]
            for nmea_sentence in nmea_sentences:
                message = str(nmea_sentence).strip() + "\r\n"

                if self.verbosity:
                    print(f"Sending message '{message}'")

                self.sock.sendto(message.encode('utf-8'), (self.ip, self.port))

        except pynmea2.nmea.ParseError as e:
            print(f"ParseError for message {e}")


class NMEAReceiver():

    def __init__(self, port, ip=None, verbosity=False):
        if ip is None:
            ip = socket.gethostbyname(socket.gethostname())

        if verbosity:
            print(ip, port)

        self.verbosity = verbosity
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"Attempting to bind at {ip}:{port}")
        self.sock.bind((ip, port))
    
    def _listen_thread(self, callback):
        while True:
            data, client = self.sock.recvfrom(1024)

            msg = data.decode('utf-8')
            msg = msg.rstrip()

            if self.verbosity:
                print(f"Received message '{msg}'")

            nmea = pynmea2.parse(msg)

            if self.verbosity:
                print(f"Succesfully parsed to NMEASentenceType '{type(nmea)}'")

            callback(nmea)


    def listen(self, callback):
        t = threading.Thread(target=self._listen_thread, args=(callback,))
        t.start()

if __name__ == '__main__':
    client = NMEAClient("")
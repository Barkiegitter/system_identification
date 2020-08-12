'''
Listen to UDP port, write the results to a file or simply to stdout.

Usage:
    listen_udp (<port>) [options]

Options:
    --local-ip=<s>  Denote a specific IP to listen to [Default: None]
    --parse         Parse as NMEA strings
    --binary        Write as raw bytes to file
    --verbose       Verbose output [Default: False]
    --file=<f>      Output file path [Default: None]
    --monitor       Monitor number of bytes received [Default: False]
    --recurrent     continuously recieve
'''
import socket
import pynmea2
import json


class UDPListener(object):

    def __init__(self, port, ip=None, parse=False, binary=False, file_path=None, callback=None, buffer_size=1024, verbose=False, monitor=False, recurrent=False):
        if ip is None:
            ip = socket.gethostbyname(socket.gethostname())
            print(f"Automatically resolved local IP to {ip}")

        with open('config.json', 'r') as json_file:
            config = json_file.read()
        settings = json.loads(config)
        self.cambpix = settings['cambpix'] 

        self.port = port
        self.ip = ip
        self.parse = parse
        self.binary = binary
        self.buffer_size = buffer_size
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.callback = callback
        self.verbose = verbose
        self.monitor = monitor
        self.recurrent = recurrent        

        if file_path:
            if binary:
                self.out_file = open(file_path, 'wb+')
            else:
                self.out_file = open(file_path, 'w+')
        else:
            self.out_file = None

        self.run()

    def process_block(self, block):

        #this is to strip the Cambridge Pixel 16 byte header from the AIS string
        
        out = block.rstrip()
        

        if self.parse:
            out = block.decode('utf-8').rstrip()
            parsed = pynmea2.parse(out)

            if self.binary:  
                out = str.encode(str(parsed))
            else:
                out = str(parsed)


        elif not self.binary:
            try:
                out = block.decode('utf-8')  #GK
            except:
                failed  = 1 ###Just putting things here for now
        else:
            out = block

        if self.out_file:
            self.out_file.write(out)

        self.nmea_message = out.rstrip()
        # Only print to stdout if verbose or no output file
        
        # if self.verbose or not self.out_file:
        #     print(self.nmea_message)


        if self.callback:
            self.callback(out)

    def nmea_sentence_spx(self, ):
        bytes_read = 0
        data, client = self.sock.recvfrom(self.buffer_size)
        bytes_read += len(data)

        # SPx AIS 16 header bytes and  \x00 and a carriage return at the end of string - get rid of it
        # SPx NMEA has \x00 and carriage return at the end of string - get rid of it        
        if self.cambpix and data[0]== ord('$'):
            data = data.partition(b'\0')[0].decode('utf-8').strip()
        else:
            data = data[16:].partition(b'\0')[0].decode('utf-8').strip()

        if '$PSTT' in data:
            data = ""
                

        if self.monitor:
            print(f"\rTotal bytes read: {bytes_read / (1024 * 1024):.2f} MB", end="")

        
        self.process_block(data)
        
        return(self.nmea_message)
    
    def nmea_sentence(self, ):
        bytes_read = 0
        data, client = self.sock.recvfrom(self.buffer_size)
        bytes_read += len(data)


        self.process_block(data)
        
        return(self.nmea_message)

        

    def run(self):
        if self.parse and self.binary:
            print("Cannot use parse and binary format at the same time")

        print(f"Attempting to bind at {self.ip}:{self.port} ... ", end="")
        self.sock.bind((self.ip, self.port))
        print(f"Success! Listening ... ")

        if self.recurrent:        
            while True:
                try:
                    self.nmea_sentence()
                    
                except KeyboardInterrupt as e:
                    print(e)
                    print("Cleaning up after interrupt...")
        else:
            self.nmea_sentence()

# if __name__ == '__main__':
#     # arguments = docopt(__doc__, version='0.1')
#     # print(arguments)

#     # ip = arguments['--local-ip']
#     # if ip == "None":
#     #     ip = None

#     # port = int(arguments['<port>'])
#     # file = arguments['--file']
#     # if file == "None":
#     #     file = None

#     # binary = arguments['--binary']
#     # parse = bool(arguments['--parse'])
#     # verbose = bool(arguments['--verbose'])
#     # monitor = bool(arguments['--monitor'])
#     # recurrent = bool(arguments['--recurrent'])

#     udp_listener = UDPListener(port, ip, parse, binary, file, None, 1024, verbose, monitor, recurrent)
    
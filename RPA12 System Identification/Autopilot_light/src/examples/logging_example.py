import logging
import time

from src import logger

from multiprocessing import Process, Queue

q = Queue()

def process_one():

    logger.worker_configurer(self.settings['LOGGER'])
    log = logging.getLogger('ShipShape')

    while True:
        log.info("Hello from process one")
        time.sleep(1)


def process_two():

    logger.worker_configurer(self.settings['LOGGER'])
    log = logging.getLogger('ShipShape')

    while True:
        msg = "Hello hello from process two"
        print(msg)
        log.info(msg)
        time.sleep(1)

p1 = Process(target=process_one)
p2 = Process(target=process_two)
logging_proc = Process(name="Logger_Listener", target=logger.LoggerListener,
                       args=(q,))
logging_proc.deamon = True
logging_proc.start()
p1.daemon = True
p2.daemon = True
p1.start()
p2.start()

while True:
    time.sleep(1)
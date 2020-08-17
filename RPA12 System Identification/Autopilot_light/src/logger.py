import logging, os, time, sys
import logging.handlers
from multiprocessing import Process, Queue

""""
This code is based on the example provided by the PYTHON community wit a small number of additions.
"""""


def listener_configurer(path):
    """
    This function configures the logger settings for the logger ``thread``.
    Two different handlers are created:
    * consoleHandler which modifies ``logging.StreamHandler()``
    * A root logger which outputs to a file (log/mptest.log) by default

    :param path: Path to the directory where the logfile will be stored
    :type path: str
    """
    log_file = 'log'
    if not os.path.exists(path):  # not sure if this works in windows previously /log
        os.makedirs(path)
    # Write execution line
    with open("%s/%s.log" % (path, log_file), 'a') as f:
        f.write('___________START_OF_LOGGING___________\n')
    f.close()

    root = logging.getLogger()
    root.handlers.pop()
    h = logging.FileHandler('%s/%s.log' % (path, log_file), 'a')
    f = logging.Formatter('%(asctime)s %(processName)-10s %(name)s %(levelname)-8s %(message)s')
    c_f = logging.Formatter('%(asctime)s %(processName)-10s %(name)s %(levelname)-8s %(message)s', '%H:%M:%S')
    h.setFormatter(f)
    root.addHandler(h)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.INFO)
    consoleHandler.setFormatter(c_f)
    root.addHandler(consoleHandler)


# This is the listener process top-level loop: wait for logging events
# (LogRecords)on the queue and handle them, quit when you get a None for a
# LogRecord.

# default_queue = Queue()
# queue = Queue()

def default():
    settings = {"root_name": "Listener",
                "root_level": logging.DEBUG,
                "file_path": "Log"}
    return settings


class LoggerListener(Process):
    def __init__(self, queue, settings=None):
        super(LoggerListener, self).__init__(daemon=True)
        self.queue = queue
        self.log = None
        if not isinstance(settings, dict):
            print("No dictionary passed for initialisation Defaulting to default()")
            self.settings = default()
        else:
            self.settings = settings
        print(__name__)
        self.run()

    def configurer(self):
        """
        This function configures the logger settings for the logger ``thread``.
        Two different handlers are created:
        * consoleHandler which modifies ``logging.StreamHandler()``
        * A root logger which outputs to a file (log/mptest.log) by default

        :param path: Path to the directory where the logfile will be stored
        :type path: str
        """

        path = self.settings['file_path']
        level = self.settings['root_level']
        log_file = 'log'
        if not os.path.exists(path):  # not sure if this works in windows previously /log
            os.makedirs(path)
        # Write execution line
        with open("%s/%s.log" % (path, log_file), 'a') as f:
            f.write('___________START_OF_LOGGING___________\n')
        f.close()

        root = logging.getLogger()
        try:
            root.handlers.pop()
        except IndexError:
            pass
        h = logging.FileHandler('%s/%s.log' % (path, log_file), 'a')
        f = logging.Formatter('%(asctime)s %(processName)-10s %(name)s %(levelname)-8s %(message)s')
        c_f = logging.Formatter('%(asctime)s %(processName)-10s %(name)s %(levelname)-8s %(message)s', '%H:%M:%S')
        h.setFormatter(f)
        h.setLevel(level)
        root.addHandler(h)
        consoleHandler = logging.StreamHandler()
        consoleHandler.setLevel(level)
        consoleHandler.setFormatter(c_f)
        root.addHandler(consoleHandler)
        print('Logger Reporting from PID: %s' % os.getpid())

    def run(self):
        self.configurer()
        self.log = logging.getLogger(name="Logger")
        self.log.info("Logger Reporting from PID %s" % os.getpid())
        while True:
            try:
                record = self.queue.get(block=True)
                if record is None:  # We send this as a sentinel to tell the listener to quit.
                    self.log.info("Received 'NONE' in Queue, shutting down...")
                    break
                # print(record)
                else:
                    logger = logging.getLogger(record.name)

                logger.handle(record)  # No level or filter logic applied - just do it!
            except LookupError:
                import sys, traceback
                print('Whoops! Problem:', file=sys.stderr)
                traceback.print_exc(file=sys.stderr)


class LoggerWriter:
    def __init__(self, level):
        # self.level is really like using log.debug(message)
        # at least in my case
        self.level = level

    def write(self, message):
        # if statement reduces the amount of newlines that are
        # printed to the logger
        # print('test')
        if message != '\n':
            self.level(message)

    def flush(self):
        # create a flush method so things can be flushed when
        # the system wants to. Not sure if simply 'printing'
        # sys.stderr is the correct way to do it, but it seemed
        # to work properly for me.
        self.level(sys.stderr)


def worker_configurer(settings):
    """
    This function is sent as a reference to all **Processes** and **threads** which use this function to link
    their loggers.

    :param queue: The queue used as a handler for the main logger to read.
    :type queue: multiprocessing.queue
    """
    h = logging.handlers.QueueHandler(settings['log_q'])  # Just the one handler needed

    root = logging.getLogger()
    try:
        root.handlers.pop()
    except IndexError:
        pass
    root.addHandler(h)
    # send all messages, for demo; no other level or filter logic applied.
    # sys.stdout = LoggerWriter(root.debug)
    sys.stderr = LoggerWriter(root.warning)
    root.setLevel(settings['root_level'])


if __name__ == "__main__":
    from multiprocessing import Queue

    log_q = Queue()
    b = default()
    a = LoggerListener(log_q, b)
    a.run()

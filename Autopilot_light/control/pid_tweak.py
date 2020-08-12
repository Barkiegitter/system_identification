import logging
import time
from cap_comm import ShipCommClient
from cap_comm.ship_comm_client.constants import DataType

def update_PID():
    """
    Generate a simple PID/heading message
    :return:
    """
    a = input("Update: <P/I/D/O/H>:<val/val/val/mM/val>:")
    a = a.split(':')
    idx = 0
    lst = ['P', 'I', 'D', 'O', 'H']
    try:
        idx = lst.index(a[0])
        if idx == 3:
            val = 0.
        val = float(a[1])
        client.write_topic(topic_name, f"{lst[idx]}:{val}".encode('utf8'))
    except Exception:
        print("value isn't floatable")


if __name__ == "__main__":
    start_time = time.time()
    topic_name = "pid_mv"
    client = ShipCommClient("192.168.10.4")
    client.init_read_topic(topic_name, DataType.Utf8)
    client.init_write_topic(topic_name, DataType.Utf8)
    """
    msg structure: 'P:<val>:I:<val>:D:<val>'
    """
    msg = "P:0:I:0:D:0"
    # client.write_topic(topic_name, msg.encode('utf8'))
    cmd = False
    pid_msg = None
    # while pid_msg == None:
    #     pid_msg = client.read_topic(topic_name)
    #     client.write_topic(topic_name, msg.encode('utf8'))
    #     print("Read >>>", pid_msg)
    # client.write_topic(topic_name, msg.encode('utf8'))
    while True:
        update_PID()
        time.sleep(1)
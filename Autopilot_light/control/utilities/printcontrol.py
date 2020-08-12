import matplotlib.pyplot as plt
import numpy as np
import time
from cap_comm.ship_comm_client.client import ShipCommClient, read_topic, DataType
# from src.utilities.remaining import get_ip

# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')

# def live_plotter(x_vec, y_data, ax, identifier='Real-Time Response', pause_time=0.1):
#
#     if line1 == []:
#         # this is the call to matplotlib that allows dynamic plotting
#         plt.ion()
#         fig = plt.figure(figsize=(13, 6))
#         ax = fig.add_subplot(111)
#         # create a variable for the line so we can later update it
#         plt.ylabel('heading')
#         plt.title('Title: {}'.format(identifier))
#
#     # after the figure, axis, and line are created, we only need to update the y-data
#     # line1.set_ydata(y_data)  # adjust limits if new data goes beyond bounds
#     plt.plot(x_vec, y_data)
#     # if np.min(y_comb) <= line1.axes.get_ylim()[0] or np.max(y_comb) >= line1.axes.get_ylim()[1]:
#     plt.ylim([np.min(y_data) - np.std(y_data)-10, np.max(y_data) + np.std(y_data) +10])
#     # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
#     plt.pause(pause_time)   #maybe take it out    # return line so we can update it again in the next iteration
#
#     return ax

if __name__=="__main__":
    sc = ShipCommClient("172.16.128.163",
                                        local_address='172.16.128.141',
                                        process_name="plotter")
    sc.init_read_topic('metrics', DataType.Utf8)

    x_vec = np.linspace(0, 1, 1000 )[0:-1]
    y_vec = np.zeros((7, len(x_vec)))
    t_init = time.time()

    plt.ion()
    fig = plt.figure(figsize=(16, 6))
    ax = fig.add_subplot(211)
    # create a variable for the line so we can later update it
    line_1 = ax.plot(x_vec, y_vec[0, :], '-r', alpha=1)[0]
    line_2 = ax.plot(x_vec, y_vec[1, :], '-b', alpha=1)[0]
    line_3 = ax.plot(x_vec, y_vec[2, :], '-g', alpha=1)[0]
    line_4 = ax.plot(x_vec, y_vec[3, :], '-m', alpha=1)[0]
    line_5 = ax.plot(x_vec, y_vec[4, :], '-k', alpha=1)[0]
    plt.ylabel('Control outputs')
    plt.title('Title: {}'.format('Real-Time Response'))
    plt.legend(['pid_s', 'Pv', 'Iv', 'Dv', 'Fv'])
    plt.ylim([-35, 35])
    ax2 = fig.add_subplot(212)
    line_6 = ax2.plot(x_vec, y_vec[5, :], '-g', alpha=0.8)[0]
    line_7 = ax2.plot(x_vec, y_vec[6, :], '-b', alpha=0.8)[0]
    plt.ylim([-10, 370])
    plt.legend(['Cur_H', 'Des_H'])
    plt.ylabel('Heading')
    plt.show()

    while True:
        msg = read_topic('metrics', block=True)
        if msg:
            # print('msg')
            vals = msg.split(':')
            y_vec = np.roll(y_vec, -1)
            # x_vec = np.roll(x_vec, -1)
            # x_vec[-1] = time.time()-t_init
            y_vec[:, -1] = np.array([float(vals[1]), float(vals[3]), float(vals[5]), float(vals[7]),
                                     float(vals[9]), float(vals[11]), float(vals[13])])
            line_1.set_ydata(y_vec[0, :])
            line_2.set_ydata(y_vec[1, :])
            line_3.set_ydata(y_vec[2, :])
            line_4.set_ydata(y_vec[3, :])
            line_5.set_ydata(y_vec[4, :])
            line_6.set_ydata(y_vec[5, :])
            line_7.set_ydata(y_vec[6, :])
            plt.axes(ax)
            plt.ylim([1.5*np.min(y_vec[0:4, :]), 1.5*np.max(y_vec[0:4, :])])
            fig.canvas.flush_events()
            time.sleep(.01)
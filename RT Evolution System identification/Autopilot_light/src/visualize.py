import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')


def live_plotter(x_vec, y1_data,y2_data, line1, line2, identifier='Real-Time Response', pause_time=0.1):
    if line1 == []:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = plt.figure(figsize=(13, 6))
        ax = fig.add_subplot(111)
        # create a variable for the line so we can later update it
        line1, = ax.plot(x_vec, y1_data, '-', alpha=0.8)
        line2, = ax.plot(x_vec, y2_data, '-', alpha=0.8)
        # update plot label/title
        plt.ylabel('heading')
        plt.title('Title: {}'.format(identifier))
        plt.show()

    # after the figure, axis, and line are created, we only need to update the y-data
    line1.set_ydata(y1_data)
    line2.set_ydata(y2_data)
    
    # adjust limits if new data goes beyond bounds

    y_comb = np.concatenate([y2_data,y1_data], axis=0)

    # if np.min(y_comb) <= line1.axes.get_ylim()[0] or np.max(y_comb) >= line1.axes.get_ylim()[1]:
    plt.ylim([np.min(y_comb) - np.std(y_comb)-10, np.max(y_comb) + np.std(y_comb) +10])
    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(pause_time)   #maybe take it out

    # return line so we can update it again in the next iteration
    return line1, line2




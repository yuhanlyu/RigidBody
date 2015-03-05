import matplotlib.pyplot as plt
import colorsys
import sys, getopt, gc
colors = []
x_list = {}
y_list = {}
dx_list = []
dy_list = []

def build_color(n):
    for i in range(n):
        x_list[i] = []
        y_list[i] = []
        colors.append(colorsys.hsv_to_rgb(float(i) / n, 1.0, 0.9))

def plot_from_file():
    max_number = 0
    colors = 0
    while 1:
        try:
            line = raw_input()
            x = [float(s) for s in line.split()]
            print x[0], x[1]
            number = int(x[2])
            d = x[3]
            if number not in x_list:
                x_list[number] = [x[0]]
                y_list[number] = [x[1]]
                colors = colors + 1
            else:
                x_list[number].append(x[0])
                y_list[number].append(x[1])
            if number > max_number:
                max_number = number

            step = 0.25;
            index = int(round(d / step))
            error = d - index * step
            if abs(error) < 0.02:
                dx_list.append(x[0])
                dy_list.append(x[1])
        except(EOFError):
            break

    fig, ax = plt.subplots(figsize=(5.5,5.5))
    plt.axis('equal')
    plt.rc('pdf', fonttype = 42)
    print "Uses ", colors, " colors"
    color_count = 0
    for i in range(max_number + 1):
        if i in x_list:
            color = colorsys.hsv_to_rgb(float(color_count) / colors, 1.0, 0.9)
            print "Number", i, "'s color is ", color
            ax.plot(x_list[i], y_list[i], '.', c=color, aa=True)
            color_count = color_count + 1
    ax.plot(dx_list, dy_list, '.', c='black', aa=True)
    plt.draw()
    plt.show()

if __name__ == "__main__":
    plot_from_file()

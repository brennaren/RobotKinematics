import time
import sys

import plotShapes

available_functions = ['plotShapes.plot_circle', 'plotShapes.plot_line',
                       'plotShapes.plot_quadrilateral',
                       'plotShapes.plot_curved_quadrilateral',
                       'plotShapes.plot_semicircle']
immediate_functions = ['relativeTime', 'set_up_plot']
queued_functions = []
starting = 0.0
relative = True  # true for incremental time (default), False for relative time
started = False
nextStart = 0.0


def run_function(arg):  # helper function to check if registered
    func = arg.split('(', 1)
    if func[0] in available_functions:
        exec(arg)


def relativeTime(arg):
    global relative
    relative = arg
    if relative:
        print(f"@{time.time()-starting:.2f} Interpretated as relative times")
    else:
        print(f"@{time.time()-starting:.2f}",
              "Interpretated as absolute times, starting now")


def set_up_plot():
    plotShapes.create_plot()


def start_queue():
    global started
    started = True
    startTime = time.time()
    for item in queued_functions:
        while (time.time() - startTime) < item[0]:
            time.sleep(0.01)
        print("Executing ", item[1])
        run_function(item[1])
    print(f"@{time.time()-starting:.2f} Executed all queued functions")


def main():
    global starting
    global nextStart
    print("starting main, using file list of functions")

    if len(sys.argv) == 1:
        myfile = 'instructions.txt'
    else:
        myfile = sys.argv[1]
    print("reading file ", myfile)

    with open(myfile, encoding="utf-8") as myf:
        actionList = myf.readlines()

    starting = time.time()
    for i in actionList:
        print(i, end='')
        if not i.startswith('#'):
            j = i.split(' ', 1)
            func = j[1].split('(', 1)
            if func[0] in immediate_functions:
                print("executing ", j[1])
                exec(j[1])
            else:
                if relative:
                    nextStart += float(j[0])
                else:
                    nextStart = float(j[0])
            queued_functions.append((nextStart, j[1]))

    print("input file commands added")
    start_queue()
    plotShapes.end_plot()
    print(f"@{time.time()-starting:.2f} Completed and cleanup done")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("@{time.time()-starting:.2f} Stopped by user, cleanup done")

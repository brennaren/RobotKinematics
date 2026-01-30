import threading
import time
import sys


available_functions = ['hello', 'ended']
immediate_functions = ['relativeTime', 'startQ']
starting = 0.0
relative = True  # true for incremental time (default), False for relative time
threads = []
started = False
nextStart = 0.0


def hello(arg):  # dummy routine 1
    global starting
    print(f"@{time.time()-starting:.2f} hello:", arg)


def ended(arg):  # dummy routine 2
    global starting
    print(f"@{time.time()-starting:.2f} last?", arg)


def run_a_func_from_string(arg):  # helper function to check if registered
    func = arg.split('(', 1)
    if func[0] in available_functions:
        exec(arg)


def relativeTime(arg):
    global relative
    global starting
    relative = arg
    if relative:
        print(f"@{time.time()-starting:.2f} Interpretated as relative times")
    else:
        print(f"@{time.time()-starting:.2f}",
              "Interpretated as absolute times, starting now")


def startQ(arg):
    global started
    global threads
    if started:
        return
    for t in threads:
        t.start()
    started = True
    print(f"@{time.time()-starting:.2f} Executing q", arg)


def waitEndQ():
    global threads
    for t in threads:
        t.join()


def main():
    global starting
    global started
    global relative
    global threads
    global nextStart
    print("starting main, using file list of functions")

    if len(sys.argv) == 1:
        myfile = 'greetings.txt'
    else:
        myfile = sys.argv[1]
    print("reading file ", myfile)

    with open(myfile, encoding="utf-8") as myf:
        actionList = myf.readlines()

    starting = time.time()
    startAt = 0
    for i in actionList:
        print(i, end='')
        startAt = nextStart
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
                t = threading.Timer(startAt, run_a_func_from_string, [j[1]])
                threads.append(t)  # keep a list of threads
                if started:  # so actions can be added after queue started.
                    t.start()

    print("input file commands added")
    time.sleep(5)  # to see if q started by immediate command
    if not started:
        startQ("")  # argument needed
    waitEndQ()
    # stop_car()  # stop movement
    # destroy()   # clean up GPIO
    print(f"@{time.time()-starting:.2f} Completed and cleanup done")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        # stop_car() # stop movement
        # destroy()  # clean up GPIO
        print("@{time.time()-starting:.2f} Stopped by user, cleanup done")

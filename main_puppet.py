import os
import threading
from naoqi import ALProxy
import time


def start_working():

    def worker1():
        os.system('roslaunch skeleton_markers markers.launch')
        return

    def worker2():
        os.system('python curious_game/skeleton_angles_for_puppet.py')

    def worker3():
        os.system('python curious_game/servo_control.py')
        return

    t1 = threading.Thread(target=worker1)
    t1.start()
    threading._sleep(0.2)
    t2 = threading.Thread(target=worker2)
    t2.start()
    threading._sleep(0.2)
    t3 = threading.Thread(target=worker3)
    t3.start()

start_working()

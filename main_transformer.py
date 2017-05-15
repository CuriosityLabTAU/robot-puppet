import random
import time
from curious_game.transformer import Transformer
import os
import naoqi
from naoqi import ALProxy

print('Dont forget to run:')
print('roslaunch my_skeleton_markers markers.launch &')
print('python skeleton_markers_reader.py')
print('-------')

# basic system parameters
sleep_time = 0.25
round_duration = 5
time_steps = int(float(round_duration) / sleep_time)

# initialization
transformer = Transformer()
transformer.howie.move_to_pose(transformer.howie.base_pose)

ip = "192.168.0.100"
port = 9559
motionProxy = ALProxy("ALMotion", ip, port)
#motionProxy.get

tts = ALProxy("ALTextToSpeech", "192.168.0.100", 9559)
tts.say("Hello, world!")




# introduction
# transformer.howie.play_file('detection instruction.wav')
transformer.howie.move_to_pose(transformer.howie.poses['both_hands_up'])
transformer.child.current_state = 'wait_for_start_pose'
while transformer.child.current_state != 'start_pose_detected':
    time.sleep(sleep_time)
    pass



# starting to play
number_of_rounds = 10



# for a in range(number_of_rounds):
transformer.child.current_state = 'wait_for_current_pose'
for t in range(time_steps):
    time.sleep(sleep_time)

    # checking for updates from kinect
    if transformer.child.current_state == 'received_pose':
        # detected a pose
        print (transformer.get_pose_detected_names())
        pose_detected = "right_hand_up"
        transformer.robot_performs_action(pose_detected)



print("finish")
# simon.howie.play_file('bye.wav')
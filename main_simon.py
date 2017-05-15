import random
import time
from game.simon import Simon
import os

print('Dont forget to run:')
print('roslaunch my_skeleton_markers markers.launch &')
print('python skeleton_markers_reader.py')
print('-------')

#l = kinect.skeleton_markers_reader.kinect_listener()
os.system('kinect/skeleton_markers_reader.py')
# basic system parameters
sleep_time = 0.25
round_duration = 5
time_steps = int(float(round_duration) / sleep_time)

# initialization
simon = Simon()
simon.howie.move_to_pose(simon.howie.base_pose)

# introduction
simon.howie.play_file('detection instruction.wav')
simon.howie.move_to_pose(simon.howie.poses['both_hands_up'])
simon.child.current_state = 'wait_for_start_pose'
while simon.child.current_state != 'start_pose_detected':
    time.sleep(sleep_time)
    pass

simon.howie.play_file('intro.wav')
simon.child.current_state = 'wait_for_yes_pose'
while simon.child.current_state != 'yes_pose_detected':
    time.sleep(sleep_time)
    pass

# starting to play
number_of_rounds = 10

for a in range(number_of_rounds):
    # select a random action, and random hertzel says
    simon.robot_performs_action()
    simon.child.current_state = 'wait_for_current_pose'

    # how long to wait until next round
    for t in range(time_steps):
        time.sleep(sleep_time)

        # checking for updates from kinect
        if simon.child.current_state == 'received_pose':
            # detected a pose

            if simon.pose_selected in simon.get_pose_detected_names():
                # child performed the pose required
                if simon.hertzel_says:
                    simon.howie.play_file(random.choice(simon.howie.utterances['correct_pose']))
                    print('correct pose')
                else:
                    simon.howie.play_file(random.choice(simon.howie.utterances['got you']))
                    print('Got you!')
                break
            else:
                print('wrong pose')
                simon.child.current_state = 'wait_for_current_pose'
        # time passed, did the child respond correctly?
        if simon.hertzel_says:
            simon.howie.play_file(random.choice(simon.howie.utterances['didnt get it']))
            print('I said Simon says, you didnt get it')
        else:
            simon.howie.play_file(random.choice(simon.howie.utterances['good job']))
            print('goob job, didnt fool you')

simon.howie.play_file('bye.wav')
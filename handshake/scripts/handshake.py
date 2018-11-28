#!/usr/bin/env python
import rospy
import intera_interface
import json
import sys
import time

FILE_NAME = '/tmp/joints.json'

def loop(limb):
    positions = []

    while True:
        print('Input:')
        inp = raw_input()
        if inp == "exit":
            return positions
        elif inp == "save":
            positions.append(limb.joint_angles())
            continue

        joint, change = inp.split()
        change = float(change)
        joint = "right_j{}".format(joint)

        angles = limb.joint_angles()
        angles[joint] += change
        limb.move_to_joint_positions(angles)


def record(limb):
    with open(FILE_NAME, 'w') as fh:
        positions = loop(limb)
        fh.write(json.dumps(positions))


def playback(limb):
    with open(FILE_NAME) as fh:
        positions = json.loads(fh.read())
        for i, position in enumerate(positions):
            print('Moving to position: {}'.format(i+1))
            limb.move_to_joint_positions(position)
            time.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('handshake')
    print('Node initialized')
    limb = intera_interface.Limb('right')
    if len(sys.argv) > 1 and sys.argv[1] == 'record':
        record(limb)
    else:
        playback(limb)

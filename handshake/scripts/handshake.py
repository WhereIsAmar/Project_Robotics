#!/usr/bin/env python
import rospy
import intera_interface
import json
import sys
import time
from pprint import pprint

FILE_NAME = '/tmp/joints.json'

def loop2(limb):
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


def loop(limb):
    inp = None
    positions = []
    while inp != "q":
        print("Press enter to record position...")
        inp = raw_input()
        angles = limb.joint_angles()
        positions.append(angles)

    return positions


def record(limb):
    with open(FILE_NAME, 'w') as fh:
        positions = loop(limb)
        fh.write(json.dumps(positions))


def add_waypoint(limb, positions, i):
    print('Press key to continue...')
    raw_input()

    positions.insert(i+1, limb.joint_angles())


def delete_waypoint(limb, positions, i):
    del positions[i]


options = {
    'd': delete_waypoint,
    'e': add_waypoint
}


def playback(limb):
    limb.move_to_neutral()
    with open(FILE_NAME) as fh:
        positions = json.loads(fh.read())
        updated = positions[:]

        for i, position in enumerate(positions):
            print('Moving to position: {}'.format(i+1))
            limb.move_to_joint_positions(position)
            print('Press key to continue...')
            inp = raw_input()
            if inp in options:
                options[inp](limb, updated, i)

    print('exit')


if __name__ == '__main__':
    rospy.init_node('handshake')
    print('Node initialized')
    limb = intera_interface.Limb('right')
    limb.move_to_neutral()
    if len(sys.argv) > 1 and sys.argv[1] == 'record':
        record(limb)
    else:
        # pass
        playback(limb)

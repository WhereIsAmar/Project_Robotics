#!/usr/bin/env python
import rospy
import intera_interface
import json
import sys


def file_path(name):
    return "/home/jon/Project_Robotics/data/{}".format(name.lower())


def record(limb):
    FILE_NAME = "TODO"
    with open(FILE_NAME, 'w') as fh:
        inp = None
        positions = []
        while inp != "q":
            print("Press enter to record position...")
            inp = raw_input()
            angles = limb.joint_angles()
            positions.append(angles)

        fh.write(json.dumps(positions))


def playback(hd, limb, name):
    limb.move_to_neutral()
    hd.display_image(file_path("{}.jpg".format(name)))

    with open(file_path("{}.json".format(name))) as fh:
        positions = json.loads(fh.read())

        for i, position in enumerate(positions):
            print('Moving to position: {}'.format(i+1))
            limb.move_to_joint_positions(position)

    print('exit')


def init_sawyer():
    rospy.loginfo("initializing sawyer...")

    limb = intera_interface.Limb('right')
    hd = intera_interface.HeadDisplay()
    head = intera_interface.Head()

    limb.move_to_neutral()
    head.set_pan(0.0)

    rospy.loginfo("sawyer initialized")

    return limb, hd, head


if __name__ == '__main__':
    rospy.init_node('handshake')
    print('Node initialized')

    limb, hd, _ = init_sawyer()

    if len(sys.argv) > 1 and sys.argv[1] == 'record':
        record(limb)
    else:
        playback(hd, limb, sys.argv[2])

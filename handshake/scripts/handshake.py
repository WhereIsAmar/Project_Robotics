#!/usr/bin/env python
import rospy
import intera_interface
import json
import sys
import time


def file_path(name):
    """ get the file path to data files.

    Note: temporarily hard coded for jon's computer
    """
    return "/home/jon/Project_Robotics/data/{}".format(name.lower())


def record(limb, name):
    """ record a new handshake """
    path = file_path("%s.json" % name)

    with open(path, 'w') as fh:
        movements = []
        while True:
            print("Press enter to record position...")
            if raw_input() == 'q':
                break

            movement = {
                "angles": limb.joint_angles(),
                "delay": 0.0
            }
            movements.append(movement)

        fh.write(json.dumps(movements))

    rospy.loginfo('recording complete. file saved at {}'.format(path))


def playback(hd, limb, name):
    """ playback a handshake """
    rospy.loginfo('playing %s' % name)
    limb.move_to_neutral(timeout=1)
    hd.display_image(file_path("{}.jpg".format(name)))

    with open(file_path("{}.json".format(name))) as fh:
        movements = json.loads(fh.read())

        for i, movement in enumerate(movements):
            rospy.loginfo('Moving to position: {}'.format(i+1))
            limb.move_to_joint_positions(movement['angles'], timeout=0.5)

            rospy.loginfo('pausing {:.1f} seconds'.format(movement['delay']))
            time.sleep(movement['delay'])

    limb.move_to_neutral(timeout=1)
    rospy.loginfo('playback done')


def init_sawyer():
    """ basic setup stuff """
    rospy.loginfo("initializing sawyer...")

    limb = intera_interface.Limb('right')
    hd = intera_interface.HeadDisplay()
    head = intera_interface.Head()

    limb.move_to_neutral(timeout=1)
    head.set_pan(0.0, timeout=1)

    rospy.loginfo("sawyer initialized")

    return limb, hd, head


if __name__ == '__main__':
    rospy.init_node('handshake')
    rospy.loginfo('handshake node initialized')

    limb, hd, _ = init_sawyer()

    if len(sys.argv) <= 1:
        print('usage: rosrun handshake handshake.py <name>')
        exit(1)

    if sys.argv[1] == 'record':
        record(limb, sys.argv[2])
    else:
        playback(hd, limb, sys.argv[1])

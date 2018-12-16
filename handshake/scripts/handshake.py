#!/usr/bin/env python
import rospy
import intera_interface
import json
import sys
import time
import os
import std_msgs


# simulator doesn't play nice and best to timeout moves. default 15.0
TIMEOUT = 1.0


def file_path(name):
    """ get the file path to data files. """
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_dir, 'data', name.lower())


def record(limb, name):
    """ record a new handshake """
    path = file_path("%s.json" % name)

    with open(path, 'w') as fh:
        movements = []
        while True:
            print("Press enter to record position (name, delay) or q to quit...")
            inp = raw_input()
            if inp == 'q':
                break

            name, delay = inp.split()
            delay = float(delay)

            movement = {
                "angles": limb.joint_angles(),
                "delay": delay,
                "name": name
            }

            movements.append(movement)

        fh.write(json.dumps(movements))

    rospy.loginfo('recording complete. file saved at {}'.format(path))


def playback(limb, hd, name):
    """ playback a handshake """
    rospy.loginfo('playing %s' % name)
    limb.move_to_neutral(timeout=TIMEOUT)
    hd.display_image(file_path("{}.jpg".format(name)))

    with open(file_path("{}.json".format(name))) as fh:
        movements = json.loads(fh.read())

        for i, movement in enumerate(movements):
            rospy.loginfo('Moving to position: {} {}'.format(i+1, movement['name']))
            limb.move_to_joint_positions(movement['angles'], timeout=TIMEOUT)

            rospy.loginfo('pausing {:.1f} seconds'.format(movement['delay']))
            delay = movement['delay']
            time.sleep(delay)

    rospy.loginfo('playback done')
    time.sleep(5)
    limb.move_to_neutral(timeout=TIMEOUT)


def listen(limb, hd):
    """ listen on /handshake/play topic """
    def callback(data):
        playback(limb, hd, str(data.data))

    rospy.Subscriber("handshake/play", std_msgs.msg.String, callback)
    rospy.loginfo('listening...')
    rospy.spin()


def init_sawyer():
    """ basic setup stuff """
    rospy.loginfo("initializing sawyer...")

    limb = intera_interface.Limb('right')
    hd = intera_interface.HeadDisplay()
    head = intera_interface.Head()

    limb.move_to_neutral(timeout=TIMEOUT)
    # TODO: Hardocoded to jon's computer
    hd.display_image("/home/jon/catkin_ws/src/sawyer_simulator/sawyer_gazebo/share/images/sawyer_sdk_research.png")
    head.set_pan(0.0)

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
    elif sys.argv[1] == 'listen':
        listen(limb, hd)
    else:
        playback(limb, hd, sys.argv[1])

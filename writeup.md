Names: Abdullah Bajkhaif, Amar Patel, Bandar Albegmi, Jon McMillan

Abstract
--------
In our final project, we explored robots greeting humans. Robots will play an
important role in our daily lives in the future, and it will be critical for
robots to engage with humans in a natural, playful way. Robots that do not
greet people will be seen as cold machines. Our belief is that robots that
perform personalized greetings will be treated with greater sympathy and
tolerance. Our system uses face detection to identify the individual and start
a personalized prerecorded handshake using Sawyer, a robotic arm.

Introduction and Background
---------------------------
Human-robot interaction (HRI) is the study of interactions between humans and
robots. Most human behavior is automatic, unconscious and taken for granted.
Robots are not naturally social and must be designed to adopt social norms in
human environments. Robots that deviate from social behaviors that humans take
for granted can make some people uneasy.

In all cultures, there exist commonly used greetings. Greetings are how humans
acknowledge the presence of each other. Handshakes are a common greeting in
many cultures, and one that can be performed by a robotic arm. Handshakes vary
by culture so it's important to be able to personalize and perform a variety of
shakes.

Team Organization
------------------
Our team consistently communicated over several mediums with both our professor and assistant graduate student to organize meetings and interaction with the Sawyer Arm. Our group met several times outside of the allotted time for the lab to coordinate learning with the ROS environment. 

In order to remain focused on the goals, we had our team focused on micro deliverables to maximize efficiency. Coupled with this we always attempted to promote a healthy atmosphere for self-discovery and made it a priority to make sure everyone's ideas were heard.

Methods
-------
The system was comprised of 4 components: the robot, a handshake controller, a
web server, and face detection system. All work was done using Sawyer and its
software development kit (SDK). The SDK managed the kinematics and all lower
level robot management.

The handshake controller was a ROS Python script that was the interface between
the web server and face detection system and Sawyer. The controller recorded
joint angles needed to perform the handshake and any delays in the movement.
Angles were saved to a JSON file and played back from the file when a person
was recognized.

The web server allowed us to create a web page to interact with Sawyer via a
ROS topic. The web server extended the simple HTTP server supplied in Python's
standard library. Our goal with the website was to supply an attractive user
interface to manage the handshakes.

The face detection system detected faces in general and could be trained to
recognize particular people. It was built using OpenCV and Haar cascades. When
trained, the Haar cascades output an XML file that can be used for real time
detection. Training images were frames taken from the video stream coming from
our computer's camera.

Results and Discussion
----------------------
Our primary objective with this project was to gain experience with ROS, a
robotic arm, and computer vision. We can confidently say we met our objective.

The handshakes playback successfully, but the robot does not react to the
human's actions.  If we had more time, we would have looked into using the
force sensors to recognize when contact had been made. We recorded the angles
using the zero-G mode to place the arm in the desired position. It would be
interesting to explore teaching the robot the angles by demonstration.

The web server was limited in functionality due to time. It was able to
communicate with the handshake controller, but we ran into issues when the
controller was networked with Sawyer via an Ethernet cable and the web server
was on a separate network. However, it was an excellent proof of concept and
demonstration of the distributed nature of ROS and web and how they interact.
Websites controlling robots is a foreign concept to many and this project helps
us reimagine what is possible.

Our face detection system worked surprisingly well. The accuracy was reasonable
with as few as 100 images from the camera. We did not get an opportunity to use
the camera from Sawyer as the feed to the system. In the future, we'd have
Sawyer orient itself around the person to get better training images.

Overall, we achieved most of what we set out to do.

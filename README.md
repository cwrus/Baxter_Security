The Code in this repository is for the Baxter robot, in collaboration with the Kinect sensor. 
We are using OpenCV libraries for image processing. 

The goal of this project is to use the kinect to identify key items and have the baxter retreive them. 

Code contained within:
kinect_test: a ros package that when launched, will initialize a set of nodes to detect a red square in the view of the kinect, and publish the center of the object to a ros topic. WIP. 

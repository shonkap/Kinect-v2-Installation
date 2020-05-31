# Kinect-v2-Installation
This is a guide on how to get the kinect v2 working with windows 10. The kinect drivers are also included.

PROBLEM -> Cannot connect to the Kinect Studio Host Service:
service.-->>>Win+R--->services.msc--->KinectMonitor

If you tried to install the kinect drivers previously make sure you remove them fully before trying this

Installation:
	Kinect must be unplugged to start

1.  kinect v2 1905
2.  kinect v2 1405
3.  Kinect v2 sdk
4.  Kinect verifier(not need but used to check if kinect is working)
	
	Wait 2 minutes
	
  (Kinect must be plugged into a 3.0 port or a highspeed usb port)
	Plug in Kinect, it will connect and disconnect twice
	Run verifier and check that you can see the two streams
	Test the kinect studio
  
  SDK Browser has example code
  when using test code make sure that the proccess inside the visual studio program stops(check this by opening task manager and expanding the visual studio program)

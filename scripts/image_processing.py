import sys
import os
import rospy
import picamera
import picamera.array
import numpy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge


class ImageReshaper (picamera.array.PiMotionAnalysis) :
    def setup(self):
        self.image = None 

    def write(self, data):
        self.image = np.reshape(np.fromstring(data, dtype=np.uint8), (240, 320, 3))
    

def main():

    
    rospy.init_node('camera')

    image_pub = rospy.Publisher("/walkerbotweb/image_raw", Image, queue_size=1, tcp_nodelay=False)

    print "Vision Started"

    try:
        bridge = CvBridge()

        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (320, 240)
            with ImageReshaper(camera) as image_reshaper:

                    image_reshaper.setup()
                    camera.start_recording(image_reshaper, format='bgr')
                    # nonblocking wait
                    while not rospy.is_shutdown():
                        camera.wait_recording(1/100.0)

                        if image_reshaper.previous_image is not None:
                            image_message = bridge.cv2_to_imgmsg(image_reshaper.previous_image, encoding="bgr8")
                            image_pub.publish(image_message)

            camera.stop_recording()  # stop recording both the flow


        print "Shutdown Received"
        sys.exit()

    except Exception as e:
        print "Camera Error!"
        raise


if __name__ == '__main__':
    main()

A simple package to test OpenCV installation and ROS cv_bridge/image transport.
opencv_tests has two nodes: 
1. cv_test_node:
    Simply prints the opencv version and draws an image on the screen
2. cv_bridge_test:
    Subscribes to topic /usb_cam/image_raw  (publish with usb_cam package)
    converts ROS message to OpenCV image
    Draws a filled square on image
    Converts modified image back to ROS image message
    Published modified image message as topic image_output

Use rqt_image_view to see both images. If output_image is the same as the image_raw
but with a blue square drawn on it, congratulations - opencv and ROS are working.

https://github.com/lbrombach/opencv_tests.git
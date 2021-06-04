#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cuda.h>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//using namespace std;

image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;
int threshhole = 225;
int threshBand = 10;
int iteration = 0;

using namespace std;
using namespace cv;
using namespace cv::cuda;

void handle_image(const sensor_msgs::ImageConstPtr &img)
{
    Mat shrunk, gray, blur, canny, threshed;
    //convert ros image message to open-cv compatible BGR image
    Mat image = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8)->image;
    cvtColor(image, gray, CV_BGR2GRAY);

   // cv::cuda::GpuMat dst, src;

   // src.upload(image);

    //crop image .. zed rect output is 1920 x 1080 and I want bottom 2/5, so trying  0,height/5*3, width, height/5*2
    // this line for zed?  Mat cropped = gray(Rect(0,img->height*3/5, img->width ,img->height*2/5-1));
    Mat cropped = gray(Rect(0,img->height*.5, img->width ,img->height*.5-1));
   
    //scale
    resize(cropped, shrunk, Size(), .5, .5, INTER_LINEAR);
    
    //medianBlur(shrunk, blur, 5);
    GaussianBlur(shrunk, blur, Size(5, 5), 1.1);
    

    //void Canny(InputArray src, OutputArray dst, double threshold1 (lower),
    //	double threshold2 (upper), int aperatureSize=3,
    //	bool L2gradient = false)
    //Canny(blur, canny, 75, 125, 3);

    

    

    int threshSample=0;
    int sampleSize = 0;
    int STEP_SIZE = 5;

    //here we sample the entire (cropped , grayed, and blurred) image and find an average pixel value
    //then use some factor of that for the threshold. Intent to be able to adapt on the fly to changing lighing conditions
    if(iteration % 5 == 0){
    for (int col = 0; col< blur.size().width - STEP_SIZE; col+= STEP_SIZE){
        for(int row = 0; row < blur.size().height - STEP_SIZE; row+=STEP_SIZE){
            threshSample +=  (int)blur.at<uchar>(row, col);
            sampleSize ++;
        }
    }
    threshSample /= sampleSize;
    threshSample += (255 - threshSample)/3;
    //threshBand = threshSample/3;
    }
    cout<<"thresh values = "<<threshSample<<"  and band of "<<threshBand<<endl;

    threshold(blur, canny, threshSample , 255, THRESH_BINARY);
    //threshold(blur, canny, 175, 255, THRESH_BINARY_INV);


    //use cv_bridge to convert opencv image to ros image message
    //create image message
    sensor_msgs::Image::Ptr output_img;
    //convert opencv image we drew on to ROS image message
    //output_img = cv_bridge::CvImage(img->header, "bgr8", shrunk).toImageMsg();
    output_img = cv_bridge::CvImage(img->header, "mono8", canny).toImageMsg();

    cout << img->width << " x " << img->height << " .. total " << img->data.size() << " pixels"<<endl;
    cout << output_img->width << " x " << output_img->height << " .. total " << output_img->data.size() << " pixels"<<endl;
    //publish our ros image message
    image_pub.publish(output_img);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cuda_lanes");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);

    // Subscribe to input video feed and publish output video feed
    image_sub = it_.subscribe("image", 1, handle_image);
    image_pub = it_.advertise("image_output", 1);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
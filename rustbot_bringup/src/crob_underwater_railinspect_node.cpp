/*
    Authors: Andry Maykol Pinto.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
    STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
    WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

std::string s_tf_name;
ros::Publisher pub_info_camera_left ;
ros::Publisher pub_info_camera_right ;
image_transport::Publisher pub_left ;
image_transport::Publisher pub_right ;

ros::Subscriber sub_info_camera_left ;
ros::Subscriber sub_info_camera_right ;
image_transport::Subscriber sub_img_right;
image_transport::Subscriber sub_img_left;
image_transport::Subscriber sub_img_right_compressed;
image_transport::Subscriber sub_img_left_compressed;

void imageCallback_imgleft(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        sensor_msgs::Image _msg = *msg;
        _msg.header.frame_id    = s_tf_name.c_str();
        pub_left.publish(_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert frame if  - imageCallback_imgleft");
    }
}


void imageCallback_imgright(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        sensor_msgs::Image _msg = *msg;
        _msg.header.frame_id    = s_tf_name.c_str();
        pub_right.publish(_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert frame if  - imageCallback_imgright");
    }
}

void imageCallback_caminfo_left(const sensor_msgs::CameraInfoConstPtr& msg)
{
    try
    {
        sensor_msgs::CameraInfo _msg = *msg;
        _msg.header.frame_id         = s_tf_name.c_str();
        pub_info_camera_left.publish(_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert frame if  - imageCallback_caminfo_left");
    }
}


void imageCallback_caminfo_right(const sensor_msgs::CameraInfoConstPtr& msg)
{
    try
    {
        sensor_msgs::CameraInfo _msg = *msg;
        _msg.header.frame_id         = s_tf_name.c_str();
        pub_info_camera_right.publish(_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert frame if  - imageCallback_caminfo_right");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;


    if (nh.getParam("/param_change_frame_id", s_tf_name))
    {
        ROS_INFO("Got param: %s", s_tf_name.c_str());
    }
    else
    {
        ROS_INFO("param_change_frame_id not available ");
        s_tf_name = "stereo_camera";
    }

// 


    image_transport::ImageTransport it(nh);

    /// PUBLISHERS
    pub_info_camera_left   = nh.advertise<sensor_msgs::CameraInfo>("stereo_camera/left/camera_info", 1);
    pub_info_camera_right  = nh.advertise<sensor_msgs::CameraInfo>("stereo_camera/right/camera_info", 1);
    pub_left               = it.advertise("stereo_camera/left/image_raw", 1);
    pub_right              = it.advertise("stereo_camera/right/image_raw", 1);

    /// SUBSCRIBERS
    sub_img_right          = it.subscribe("stereo_aux/left/image_raw", 1, imageCallback_imgleft);
    sub_img_left           = it.subscribe("stereo_aux/right/image_raw", 1, imageCallback_imgright);
    //sub_img_right_compressed          = it.subscribe("stereo/left/image_raw/compressed", 1, imageCallback_imgleft);
    //sub_img_left_compressed           = it.subscribe("stereo/right/image_raw/compressed", 1, imageCallback_imgright);
    sub_info_camera_left   = nh.subscribe("stereo_aux/left/camera_info", 1, imageCallback_caminfo_left);
    sub_info_camera_right  = nh.subscribe("stereo_aux/right/camera_info", 1, imageCallback_caminfo_right);
    ros::spin();
}

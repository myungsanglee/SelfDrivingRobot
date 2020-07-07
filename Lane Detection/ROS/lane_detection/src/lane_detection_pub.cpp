#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "std_msgs/Int32.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "lane_detection_pub.h"
#include "pthread.h"

using namespace std;

void* stopLinePassThread(void *ret);

bool stop_line_detect = true;
bool thread_run = false;


LaneDetector::LaneDetector() : h(480), w(640)
{
}

cv::Mat LaneDetector::colorFilter(const cv::Mat &image)
{
    cv::Mat hsv, v, white, yellow, or_img;
    cv::Mat yellow_lower = (cv::Mat1d(1, 3) << 5, 90, 100);
    cv::Mat yellow_upper = (cv::Mat1d(1, 3) << 200, 255, 255);
    vector<cv::Mat> hsv_split;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::split(hsv, hsv_split);
    v = hsv_split[2];
    cv::inRange(v, 245, 255, white);
    cv::inRange(hsv, yellow_lower, yellow_upper, yellow);
    cv::bitwise_or(white, yellow, or_img);
    cv::flip(yellow, yellow, -1);
    cv::flip(white, white, -1);

    return or_img;
}

LaneDetector::warpped_ret LaneDetector::warping(const cv::Mat &image)
{
    warpped_ret r;
    cv::Mat w_img, transform_matrix, minv;
    vector<cv::Point2f> source = {cv::Point2f(0.4 * w, 0.45 * h), cv::Point2f(0.6 * w, 0.45 * h), cv::Point2f(0.0, h), cv::Point2f(w, h)};
    vector<cv::Point2f> destination = {cv::Point2f(0.27 * w, 0), cv::Point2f(0.65 * w, 0), cv::Point2f(0.27 * w, h), cv::Point2f(0.73 * w, h)};
    transform_matrix = cv::getPerspectiveTransform(source, destination);
    minv = cv::getPerspectiveTransform(destination, source);
    cv::warpPerspective(image, w_img, transform_matrix, cv::Size(w, h));
    r.w_img = w_img;
    r.minverse = minv;

    return r;
}

cv::Mat LaneDetector::roi(const cv::Mat &image)
{
    cv::Mat mask = cv::Mat::zeros(int(h), int(w), CV_8U);
    cv::Mat masked_img;
    vector<vector<cv::Point2i>> shape = { { {150, 0},{int(w) - 150, 0},{int(w) - 150, int(h)},{150, int(h)} } };
    cv::fillPoly(mask, shape, 255);
    cv::bitwise_and(image, mask, masked_img);

    return masked_img;
}

cv::Mat LaneDetector::windowRoi(const cv::Mat &binary_img, const cv::Mat &minv, int num, ros::Publisher *center_pub) 
{
    std_msgs::Int32 center_pub_val;
    cv::Mat out_img = binary_img.clone(), img_cp, out, rect_img, w_img;
    vector<int> max_wleft, max_wright;
    int max_left = 0, max_right = 0, max = 0, margin = 30, minpix = 50, thickness = 2;
    cv::Scalar left_color(255, 0, 0);
    cv::Scalar right_color(255, 0, 0);
    //vector<cv::Mat> temp = {binary_img, binary_img, binary_img};
    //merge(temp, out_img);
    const double *next;
    cv::Mat mask = cv::Mat::zeros(int(h), int(w), CV_8UC1);
    cv::Point ret[1][6];

    for (int wd=0;wd<num;wd++)
    {
        cv::Mat hist;
        double max_l = 0, max_r = 0;
        max_left = 0; max_right = 0; max = 0;
        int win_y_top = int(h)-(50*(wd+1));
        int win_y_bottom = win_y_top + 50;
        
        cv::Rect rect(0, h-50*(wd+1), w, 50);
        img_cp = binary_img(rect);
        
        for (int i = 0; i < img_cp.cols; i++)
            hist.push_back(cv::sum(img_cp.col(i))[0]);

        for (int r = 0; r < hist.rows/2; r++) 
        {
            next = hist.ptr<double>(r, 1);
            if (max_l < *next) {
                max_l = *next;
                max_left = r;
            }
        }
        
        for (int r = hist.rows/2; r < hist.rows; r++) 
        {
            next = hist.ptr<double>(r, 1);
            if (max_r < *next) {
                max_r = *next;
                max_right = r;
            }
        }
        
        max_wleft.push_back(max_left);
        max_wright.push_back(max_right);
        
        if (wd < 3)
        {
            int win_lt = max_left - margin;
            int win_lb = max_left + margin;
            int win_rt = max_right - margin;
            int win_rb = max_right + margin;
            cv::rectangle(out_img, cv::Point(win_lt, win_y_top), cv::Point(win_lb, win_y_bottom), left_color, thickness);
            cv::rectangle(out_img, cv::Point(win_rt, win_y_top), cv::Point(win_rb, win_y_bottom), right_color, thickness);
        }
    }
    
    //우회전 (왼쪽 선 참조)
    if (max_wleft[1] - max_wleft[0] > 0)
    {
        cout << "LEFT LINE" << endl;
        if (max_wleft[3] > 100)
            max = max_wleft[3];
        else
            max = max_wleft[1];
        center_pub_val.data = max + 102;
        center = max + 102;
        //cv::circle(out_img, cv::Point(center, h-150), 5, left_color, -1);
    }
    
    //좌회전 (오른쪽 선 참조)
    else if (max_wright[0] - max_wright[1] > 0)
    {
        cout << "RIGHT LINE" << endl;
        if (max_wright[3] > 0)
            max = max_wright[3];
        else
            max = max_wright[2];
        center_pub_val.data = max - 102;
        center = max - 102;
        //cv::circle(out_img, cv::Point(center, h-150), 5, right_color, -1);
    }
    
    else
    {
        cout << "BOTH" <<endl;
        if (max_wleft[3] > 100)
            max = max_wleft[3];
        else
            max = max_wleft[1];
        center_pub_val.data = max + 102;
        center = max + 102;
        //cv::circle(out_img, cv::Point(center, h-150), 5, left_color, -1);
    }
    
    /*for(int l = 0; l < 3; l++)
    {
        ret[0][l] = cv::Point(max_wleft[l], h-50*l);
    }
    for(int r = 0; r < 3; r++)
    {
        ret[0][r+3] = cv::Point(max_wright[2-r], h-50*(2-r));
    }
    //cv::rectangle(out_img, cv::Point(center - 50, h-150), cv::Point(center + 50, h - 100), cv::Scalar(255, 0, 0), thickness);
    
    const cv::Point* ppt[1] = { ret[0] };
    int npt[] = { 6 };
    cv::fillPoly(mask, ppt, npt, 1, cv::Scalar( 216, 168, 74 ), cv::LINE_8);
    cv::warpPerspective(out_img, w_img, minv, cv::Size(w, h));
    cv::warpPerspective(mask, rect_img, minv, cv::Size(w, h));
    cv::addWeighted(w_img, 1, rect_img, 0.4, 0, out);*/
    
    center_pub -> publish(center_pub_val);

    return out_img;
}

void LaneDetector::stopLine(const cv::Mat &binary_img, ros::Publisher *stop_pub)
{
    std_msgs::Int32 stop;
    int horiz_size, err = 0;
    pthread_t pth;
    int t = 0;
    int input = 0;
    cv::Mat hist, img_cp, horizontal, horizontal_img;
    
    cv::Rect rect(327 - 25, h-150, 50, 100);
    img_cp = binary_img(rect);
    horizontal = img_cp.clone();
    
    horiz_size = img_cp.cols/20;
    
    horizontal_img = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(horiz_size, 1));
    cv::erode(horizontal, horizontal, horizontal_img, cv::Point(-1, -1));
    cv::dilate(horizontal, horizontal, horizontal_img, cv::Point(-1, -1));
    
    for (int i = 0; i < horizontal.cols; i++)
        hist.push_back(cv::sum(img_cp.col(i))[0]);
	stop.data = 0;
    if(*hist.ptr<double>(hist.rows/2, 1) > 400)
    {
#ifdef DBG
	    cout << "S T O P !!" << endl;
#endif
        stop.data = 1;
        stop_pub -> publish(stop);
        stop_line_detect = false;
    }
}

void* stopLinePassThread(void *input2)
{
    int temp;
    thread_run = true;
    
    sleep(7);
    
    stop_line_detect = true;
    thread_run = false;
    
    return (void*)temp;
}

int main(int argc, char** argv)
{
    int err = 0;
    pthread_t pth;
    int input = 0;
    int t = 0;

    ros::init(argc, argv, "lane_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher img_pub = it.advertise("pi/image", 1);
    ros::Publisher center_pub = nh.advertise<std_msgs::Int32>("pi/center", 1);
    ros::Publisher stop_pub = nh.advertise<std_msgs::Int32>("pi/stop", 1);

    cv::VideoCapture cap(0);
    
    cv::VideoWriter writer;
    int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    double fps = 20.0;
    string filename = "/home/pi/test.avi";
    writer.open(filename, fourcc, fps, cv::Size(640, 480), true);

    cv::Mat mtx = (cv::Mat1d(3, 3) << 375.02024751, 0., 316.52572289, 0., 490.14999206, 288.56330145, 0., 0., 1.);
    cv::Mat dist = (cv::Mat1d(1, 5) << -0.30130634,  0.09320542, - 0.00809047,  0.00165312, - 0.00639115);
    cv::Mat newcameramtx = (cv::Mat1d(3, 3) << 273.75825806, 0., 318.4331204, 0., 391.74940796, 283.77532838, 0., 0., 1.);

    LaneDetector *lane_detector = new LaneDetector();
    LaneDetector::warpped_ret r1;
    
    sensor_msgs::ImagePtr Image;

    while(nh.ok())
    {
        
        cv::Mat src;
        
        cap.read(src);

        if(!src.empty())
        {
            //double st=static_cast<double>(cv::getTickCount());
            cv::Mat filtered_img, rotate_img, img, warped_img, minv, roi_img, window_img, out, save_img;
            
            cv::flip(src, save_img, -1);
            writer.write(save_img);
            
            //Color filter
            filtered_img = lane_detector -> colorFilter(src);

            //180도 rotation
            cv::flip(filtered_img, rotate_img, -1);

            //Fisheye lense calibration
            cv::undistort(rotate_img, img, mtx, dist, newcameramtx);
            
            //Warpped img
            r1 = lane_detector -> warping(img);
            warped_img = r1.w_img;
            minv = r1.minverse;

            //ROI img
            roi_img = lane_detector -> roi(warped_img);
            
            //Window ROI
            out = lane_detector -> windowRoi(roi_img, minv, 4, &center_pub);
            
            Image = cv_bridge::CvImage(std_msgs::Header(), "mono8", out).toImageMsg();
            img_pub.publish(Image);
            
            
            //Find Stop Line
            if (stop_line_detect)
                lane_detector -> stopLine(roi_img, &stop_pub);
            else if(!stop_line_detect && !thread_run)
            {
                if (err = pthread_create(&pth, NULL, stopLinePassThread, (void*)&input) < 0)
                {
                    perror("Thread2 error : ");
                    exit(2);
                }
                pthread_detach(pth);
            }
            if(stop_line_detect == true)
            {
#ifdef DBG
                cout << "STOP LINE DETECT = 1" << endl;
#endif
            }
            else
            {
#ifdef DBG
                cout << "STOP LINE DETECT = 0" << endl;
#endif
            }
#ifdef DBG
                cout << "THREAD RUN = " << thread_run << endl;
#endif
            cv::imshow("result", out);

            int key = cv::waitKey(2);
            if(key == 27)
                break;
        }
        ros::spinOnce();
    }
    
    writer.release();

    return 0;
}

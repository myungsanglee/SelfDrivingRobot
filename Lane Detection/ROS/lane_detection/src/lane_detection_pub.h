#ifndef LANE_DETECTION_PUB_H
#define LANE_DETECTION_PUB_H

class LaneDetector
{
    public:
        //생성자
        LaneDetector();
        
        struct warpped_ret
        {
            cv::Mat w_img;
            cv::Mat minverse;
        };

        int center = 0;
        
        //FUNCTION
        cv::Mat colorFilter(const cv::Mat &);
        cv::Mat lightRemoval(const cv::Mat &);
        warpped_ret warping(const cv::Mat &);
        cv::Mat roi(const cv::Mat &);
        cv::Mat windowRoi(const cv::Mat &, const cv::Mat &, int num, ros::Publisher *center_pub);
        void stopLine(const cv::Mat &, ros::Publisher *stop_pub);
        
    private:
        double h = 0;
        double w = 0;
};

#endif

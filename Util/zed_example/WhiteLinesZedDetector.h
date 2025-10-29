#ifndef WHITE_LINES_ZED_DETECTOR_H
#define WHITE_LINES_ZED_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "LineSizePixProviderZed.h"

using namespace cv;
using namespace std;

struct LineCandiateInSelf
{
  cv::Point2f pa;
  cv::Point2f pb;
  double score;
  bool valid;
};


//All in real-world coords on field plane in robot-centric coords
struct CornerCandiateInSelf
{
  cv::Point2f pa0;
  cv::Point2f pa1;
  cv::Point2f pb0;
  cv::Point2f pb1;
  cv::Point2f corner;
  double score;
  bool valid;
};

struct CornerResultInSelf
{
  cv::Point2f pa;
  cv::Point2f pb;
  cv::Point2f corner;
  double score;  
};

struct LineResultInSelf
{
  cv::Point2f pa;
  cv::Point2f pb;
  double score;
};

class WhiteLinesZedDetector {
public:
    WhiteLinesZedDetector(LineSizePixProviderZed& line_size_provider);
    ~WhiteLinesZedDetector();

    // Process a single frame
    cv::Mat processFrame(const cv::Mat& input_frame, std::vector<CornerResultInSelf>& valid_corners_out, std::vector<LineResultInSelf>& valid_lines_out);
    double my_norm(cv::Point2f a, cv::Point2f b);
    char get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y);
    int getRegionSum(Mat isum, int x, int y, int w, int h);
    void non_maxima_suppression(const cv::Mat& image, cv::Mat& mask, int sizex, int sizey, int threshold);
    // void get_plane_equation(double x1, double y1, double z1, 
    //                     double x2, double y2, double z2,  
    //                     double x3, double y3, double z3,
    //                     double *ra, double *rb, double *rc, double *rd); 
    float get_line2line_angle(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4);
    float get_line_magnitude(float x1, float y1, float x2, float y2);
    float get_point2line_distance(float px, float py, float x1, float y1, float x2, float y2);
    float get_segnent2segment_distance(float xa1, float ya1, float xa2, float ya2, float xb1, float yb1, float xb2, float yb2);
    void merge_two_segments(float xa1, float ya1, float xa2, float ya2, 
                        float xb1, float yb1, float xb2, float yb2, 
                        float *xr1, float *yr1, float *xr2, float *yr2); 

    void setLineSizePixPlaneApproximationCoeff(double xa, double xb, double xc, double xd, 
                                               double ya, double yb, double yc, double yd);

    void plateuThinningX(const cv::Mat& image, cv::Mat& mask);
    void plateuThinningY(const cv::Mat& image, cv::Mat& mask); 
    void drawSimpleRadarImg(void);

    // Setter for head yaw
    void setCurrentHeadYaw(double yaw) { current_head_yaw_ = yaw; }

private:
    // Add private members as needed
    cv::Mat last_processed_frame;
    cv::Mat integral_image;  // Store the integral image
    double planex_a, planex_b, planex_c, planex_d; //Plane coeffs for line width in x direction
    double planey_a, planey_b, planey_c, planey_d; //Plane coeffs for line width in y direction
    LineSizePixProviderZed& line_size_provider; // Reference to line size provider
    double current_head_yaw_ = 0.0;
};

#endif // WHITE_LINES_ZED_DETECTOR_H 
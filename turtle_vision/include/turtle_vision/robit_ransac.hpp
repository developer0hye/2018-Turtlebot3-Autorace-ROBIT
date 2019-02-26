#ifndef ROBIT_RANSAC_H
#define ROBIT_RANSAC_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <time.h>
#include <vector>


class JYJ_RansacLine{
public:

  JYJ_RansacLine(const double &success_rate = 0.99,
                 const double &inlier_rate = 0.5,
                 const double &distance_threshold = 1.0,
                 const double &length_threshold = 20.0);

  JYJ_RansacLine(const cv::Mat& Img,
                 const bool   bAutoThreshold = false,
                 const bool   is_right = false,
                 const double &success_rate = 0.99,
                 const double &inlier_rate = 0.1,
                 const double &distance_threshold = 1.0,
                 const double &length_threshold = 15.0,
                 const double length_threshold_max = 9999.0);
public:
  ~JYJ_RansacLine();

public:
  unsigned char*          m_img_data;

public:
  void                    runRansac();

  void                    setInlierRate         (const double& inlier_rate){m_inlier_rate = inlier_rate;}
  void                    setSuccessRate        (const double& success_rate){m_success_rate = success_rate;}
  void                    setLengthThreshold    (const double& length_threshold){m_length_threshold = length_threshold;}
  void                    setDistanceThreshold  (const double& distance_threshold){m_distance_threshold = distance_threshold;}
  void                    setCostThreshold      (double threshold){m_cost_threshold = threshold;}
  double                  getLineDegree         (){return m_degree;}
  cv::Point               getLineStart          (){return m_line_start;}
  cv::Point               getLineEnd            (){return m_line_end;}

private:
  cv::Mat                 m_Image;
  cv::Point               m_line_end;
  cv::Point               m_line_start;
  std::vector<cv::Point>       m_edge_point;

  int                     m_width;
  int                     m_height;
  double                  m_degree;
  double                  m_inlier_rate;
  double                  m_success_rate;
  double                  m_length_threshold;
  double                  m_length_threshold_max;
  double                  m_distance_threshold;
  double                  m_cost_threshold;
  bool                    m_is_right;
  bool                    m_bAutoThreshold;

private:
  void                    _getEdgePoint();
  unsigned int            _getMaxIteration      (const unsigned int& edge_point_size);
  bool                    _getSamples           (std::vector<cv::Point>& sample_point, const unsigned int& edge_point_size);
  const double            _getLineCost          (std::vector<cv::Point>& sample_point, const unsigned int &edge_point_size);

};

#endif // ROBIT_RANSAC_H

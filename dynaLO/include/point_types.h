/**
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lio-mapping>
* or <https://sites.google.com/view/lio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

// adapted from velodyne ros driver

#ifndef LIO_POINT_TYPES_H_
#define LIO_POINT_TYPES_H_
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

namespace lio {

/** Euclidean coordinate, including intensity and ring number. */
struct PointXYZIR
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t ring;                      ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
}
EIGEN_ALIGN16;

enum LabelType {
  NONE = 0,
  OUTLIER = 1,  
  STATIC_CAR = 5,    
  STATIC = 9,      // 
  CAR = 10,  
  BICYCLE = 11,       
  ROAD = 40,
  BUILDING = 50,  
  MOVING_CAR = 252,    
  MOVING_BICYCLE = 253,  
};

struct PointXYZIRTL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    int32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


}  // namespace lio

POINT_CLOUD_REGISTER_POINT_STRUCT(lio::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))


POINT_CLOUD_REGISTER_POINT_STRUCT(lio::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(lio::PointXYZIRTL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time)(int, label, label))


#endif  // LIO_POINT_TYPES_H_
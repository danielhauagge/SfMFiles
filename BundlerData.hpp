// Copyright (C) 2011 by Daniel Cabrini Hauagge
//
// Permission is hereby granted, free  of charge, to any person obtaining
// a  copy  of this  software  and  associated  documentation files  (the
// "Software"), to  deal in  the Software without  restriction, including
// without limitation  the rights to  use, copy, modify,  merge, publish,
// distribute,  sublicense, and/or sell  copies of  the Software,  and to
// permit persons to whom the Software  is furnished to do so, subject to
// the following conditions:
//
// The  above  copyright  notice  and  this permission  notice  shall  be
// included in all copies or substantial portions of the Software.
//
// THE  SOFTWARE IS  PROVIDED  "AS  IS", WITHOUT  WARRANTY  OF ANY  KIND,
// EXPRESS OR  IMPLIED, INCLUDING  BUT NOT LIMITED  TO THE  WARRANTIES OF
// MERCHANTABILITY,    FITNESS    FOR    A   PARTICULAR    PURPOSE    AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE,  ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#ifndef __BUNDLER_DATA_HPP__
#define __BUNDLER_DATA_HPP__

// STD
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

// Eigen3
#include <Eigen/Core>
#include <Eigen/LU>

// Boost libs
#include <boost/shared_ptr.hpp>

// Author: Daniel Cabrini Hauagge <hauagge@cs.cornell.edu>
//   Date: 2011-04-03

// Reference for file format:
//     http://phototour.cs.washington.edu/bundler/bundler-v0.4-manual.html

namespace BDATA
{
  class BadFileException: public std::runtime_error
  {
  public:
    explicit BadFileException(const std::string what): std::runtime_error(what)
    {}
  };
  
  typedef struct {
    unsigned char r, g, b;
  } Color;
  
  // Stores intrinsic and extrinsic parameters for camera
  class Camera
  {
  public:
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation; 
    double focalLength; // Focal length
    double k1, k2; // Radial distortion parameters
 
    void im2world(int imWidth, int imHeight,
		  const Eigen::Vector2d &im, Eigen::Vector3d &w) const;
    void world2im(int imWidth, int imHeight,
		  const Eigen::Vector3d &w, Eigen::Vector2d &im) const;
    
    void cam2world(const Eigen::Vector3d &c, Eigen::Vector3d &w) const;
    void world2cam(const Eigen::Vector3d &w, Eigen::Vector3d &c) const;

    void intrinsicMatrix(int imWidth, int imHeight, Eigen::Matrix3d &K) const;
    void invIntrinsicMatrix(int imWidth, int imHeight, Eigen::Matrix3d &invK) const;
       
    bool isValid() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };
    
  class PointEntry // TODO Find a better name for this class
  {
  public:
    int camera, key;
    Eigen::Vector2d keyPosition;
    
    PointEntry();
    PointEntry(int camera, int key, Eigen::Vector2d keyPosition);
        
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };
    
  // Stores point location, color and list of cameras that can view this point
  class PointInfo
  {
  public:
    Eigen::Vector3d position;
    Color color;
    std::vector<PointEntry> viewList;
        
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };
    
  // Class that represents bundler output, encapsulating
  // camera information and point information for reconstructed model.
  class BundlerData
  {
  public:
    typedef boost::shared_ptr<BundlerData> Ptr;
    static BundlerData::Ptr New(const char *bundlerFileName);
    
    BundlerData() {};
    BundlerData(const char *bundlerFileName);
    void init(const char *bundlerFileName);
        
    void readFile(const char *bundlerFileName);
    void writeFile(const char *bundlerFileName, bool ASCII = true) const;
    
    int getNCameras() const { return _cameras.size(); }
    int getNPoints() const { return _points.size(); }
    
    const std::vector<PointInfo> &getPointInfo() const { return _points; };
    const std::vector<Camera> &getCameras() const { return _cameras; };
    
    std::vector<PointInfo> &getPointInfo() { return _points; };
    std::vector<Camera> &getCameras() { return _cameras; };
    
  private:
    static const char *BINARY_SIGNATURE;
    static const char *ASCII_SIGNATURE;

    std::vector<Camera> _cameras;
    std::vector<PointInfo> _points;
    
    void readFileASCII(const char *bundlerFileName);        
    void readFileBinary(const char *bundlerFileName);
    
    void writeFileASCII(const char *bundlerFileName) const;        
    void writeFileBinary(const char *bundlerFileName) const;
  };
}
//
//std::istream& operator>> (std::istream &s, BDATA::Camera &cam);
//std::istream& operator>> (std::istream &s, Eigen::Vector2d &p);
//std::istream& operator>> (std::istream &s, Eigen::Vector3d &p);
//std::istream& operator>> (std::istream &s, Eigen::Matrix3d &m);
//std::istream& operator>> (std::istream &s, BDATA::Color &c);

#endif

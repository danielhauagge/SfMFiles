#ifndef __BUNDLER_DATA_HPP__
#define __BUNDLER_DATA_HPP__

// STD
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

// Eigen3
#include <Eigen/Core>
#include <Eigen/LU>

// Boost libs
#include <boost/shared_ptr.hpp>

// Project includes
#include "Macros.hpp"

// Author: Daniel Cabrini Hauagge <hauagge@cs.cornell.edu>
//   Date: 2011-04-03

// Reference for file format:
//     http://phototour.cs.washington.edu/bundler/bundler-v0.4-manual.html

namespace BDATA
{
    // TODO use Eigen3 types instead of double arrays
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
        
        void cam2world(const Eigen::Vector3d &c, Eigen::Vector3d &w) const;
        void world2cam(const Eigen::Vector3d &w, Eigen::Vector3d &c) const;
        
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
        
        void writeToFile(const char *bundlerFileName);
        
        int getNCameras() const { return _cameras.size(); }
        int getNPoints() const { return _points.size(); }
        
        const std::vector<PointInfo> &getPointInfo() const { return _points; };
        const std::vector<Camera> &getCameras() const { return _cameras; };

        std::vector<PointInfo> &getPointInfo() { return _points; };
        std::vector<Camera> &getCameras() { return _cameras; };

    private:
        std::vector<Camera> _cameras;
        std::vector<PointInfo> _points;
    };
}
//
//std::istream& operator>> (std::istream &s, BDATA::Camera &cam);
//std::istream& operator>> (std::istream &s, Eigen::Vector2d &p);
//std::istream& operator>> (std::istream &s, Eigen::Vector3d &p);
//std::istream& operator>> (std::istream &s, Eigen::Matrix3d &m);
//std::istream& operator>> (std::istream &s, BDATA::Color &c);

#endif

#ifndef __BUNDLER_DATA_HPP__
#define __BUNDLER_DATA_HPP__

#include <vector>
#include <fstream>
#include <iostream>

#include <boost/shared_ptr.hpp>

// Author: Daniel Cabrini Hauagge <hauagge@cs.cornell.edu>
//   Date: 2011-04-03

// Reference for file format:
//     http://phototour.cs.washington.edu/bundler/bundler-v0.4-manual.html

namespace BDATA
{
    // TODO use Eigen3 types instead of double arrays
    typedef double Point2[2];
    typedef double Point3[3];
    typedef double Matrix3x3[3][3];
    typedef unsigned char Color[3];
    
    // Stores intrinsic and extrinsic parameters for camera
    class Camera
    {
    public:
        Point3 translation;
        Matrix3x3 rotation; 
        double focalLength; // Focal length
        double k1, k2; // Radial distortion parameters
        
        void cam2world(const Point3 &c, Point3 &w) const;
        void world2cam(const Point3 &w, Point3 &c) const;
    };
    
    class PointEntry // TODO Find a better name for this class
    {
    public:
        int camera, key;
        Point2 keyPosition;
    };
    
    // Stores point location, color and list of cameras that can view this point
    class PointInfo
    {
    public:
        Point3 position;
        Color color;
        std::vector<PointEntry> viewList;
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

std::istream& operator>> (std::istream &s, BDATA::Camera &cam);
std::istream& operator>> (std::istream &s, BDATA::Point2 &p);
std::istream& operator>> (std::istream &s, BDATA::Point3 &p);
std::istream& operator>> (std::istream &s, BDATA::Matrix3x3 &m);
std::istream& operator>> (std::istream &s, BDATA::Color &c);

#endif

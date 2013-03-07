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

#include <sfmfiles>

// Author: Daniel Cabrini Hauagge <hauagge@cs.cornell.edu>
//   Date: 2011-04-03

// Reference for file format:
//     http://phototour.cs.washington.edu/bundler/bundler-v0.4-manual.html

namespace BDATA
{
    typedef struct {
        unsigned char r, g, b;
    } Color;
    
    // Stores intrinsic and extrinsic camera parameters
    class Camera
    {
    public:
        typedef std::vector<Camera, Eigen::aligned_allocator<Camera> > Vector;
        
        // Indexes of visible points, not stored in bundle file and only 
        // computed if extra flag is passed to BundlerData constructor.
        std::vector<int, Eigen::aligned_allocator<unsigned int> > visiblePoints; 
        
        // Extrinsic parameters
        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation; 
        
        // Intrinsic parameters
        double focalLength; // Focal length
        double k1, k2; // Radial distortion parameters
        
        // Coordinate transforms
        void im2world(const Eigen::Vector2d &im, Eigen::Vector3d &w, int imWidth = 0, int imHeight = 0) const;
        void world2im(const Eigen::Vector3d &w, Eigen::Vector2d &im, bool applyRadialDistortion = false, int imWidth = 0, int imHeight = 0) const;
        void im2cam(const Eigen::Vector2d &im, Eigen::Vector3d &c, int imWidth = 0, int imHeight = 0) const;
        void cam2im(Eigen::Vector3d c, Eigen::Vector2d &im, bool applyRadialDistortion, int imWidth = 0, int imHeight = 0) const;
        void cam2world(const Eigen::Vector3d &c, Eigen::Vector3d &w) const;
        void world2cam(const Eigen::Vector3d &w, Eigen::Vector3d &c) const;
        
        // Intrinsic matrix given image width and height
        void intrinsicMatrix(int imWidth, int imHeight, Eigen::Matrix3d &K) const;
        void invIntrinsicMatrix(int imWidth, int imHeight, Eigen::Matrix3d &invK) const;
        
        bool isValid() const;
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    
    class PointEntry // TODO Find a better name for this class
    {
    public:
        typedef std::vector<PointEntry, Eigen::aligned_allocator<PointEntry> > Vector;
        
        int camera, key; // Camera and keypoint indexes
        Eigen::Vector2d keyPosition; // Keypoint position in image
        
        PointEntry();
        PointEntry(int camera, int key, Eigen::Vector2d keyPosition);
        PointEntry(const PointEntry& other);
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    
    // Stores point location, color and list of cameras that can view this point
    class PointInfo
    {
    public:
        typedef std::vector<PointInfo, Eigen::aligned_allocator<PointInfo> > Vector;
        
        Eigen::Vector3d position;
        Color color;
        PointEntry::Vector viewList;
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    
    // Class that represents bundler output, encapsulating
    // camera information and point information for reconstructed model.
    class BundlerData
    {
    public:
        static const char* BINARY_SIGNATURE;
        static const char* ASCII_SIGNATURE;
        
        typedef boost::shared_ptr<BundlerData> Ptr;
        static BundlerData::Ptr New(const char *bundlerFileName, bool computeCam2PointIndex = false);
        
        BundlerData():_nValidCams(0), _cam2PointIndexInitialized(false) {};
        BundlerData(const char* bundleFileName, const char* listFName, bool computeCam2PointIndex = false);
        BundlerData(const char* bundleFileName, bool computeCam2PointIndex = false);
        BundlerData(const Camera::Vector& cameras, const PointInfo::Vector& points);
        
        void init(const char* bundleFileName, const char* listFName, bool computeCam2PointIndex = false);
        void init(const char* bundleFileName, bool computeCam2PointIndex = false);
        void init(const Camera::Vector& cameras, const PointInfo::Vector& points);
        
        //! Load file with image filenames
        void loadListFile(const char* listFName);
        const char* getListFileName() const;
        const char* getBundleFileName() const { return _bundleFName.c_str(); };
        //const char* getImageFileName(int camIdx) const;
        
        //! Input/Output
        void readFile(const char* bundlerFileName, bool computeCam2PointIndex = false);
        void writeFile(const char* bundlerFileName, bool ASCII = true) const;
        
        int getNCameras() const { return _cameras.size(); }
        int getNValidCameras() const { return _nValidCams; }
        int getNPoints() const { return _points.size(); }
        
        void buildCam2PointIndex();    
        
        //! Accessors
        const PointInfo::Vector& getPointInfo() const { return _points; };    
        const Camera::Vector& getCameras() const { return _cameras; };
        const std::vector<std::string>& getImageFileNames() const { return _imageFNames; };
        PointInfo::Vector& getPointInfo() { return _points; };    
        Camera::Vector& getCameras() { return _cameras; };
        std::vector<std::string>& getImageFileNames() { return _imageFNames; };
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
    private:
        Camera::Vector _cameras;
        PointInfo::Vector _points;
        int _nValidCams;
        bool _cam2PointIndexInitialized; // Weather or not to compute index Camera to visible points
        
        std::string _listFName, _bundleFName;
        std::vector<std::string> _imageFNames;
        
    protected:
        void _readFileASCII(const char* bundlerFileName);        
        void _readFileBinary(const char* bundlerFileName);
        
        void _writeFileASCII(const char* bundlerFileName) const;        
        void _writeFileBinary(const char* bundlerFileName) const;
        
        void _updateNValidCams();
    };
}

std::istream& operator>> (std::istream& s, BDATA::Camera& cam);
//std::istream& operator>> (std::istream &s, Eigen::Vector2d &p);
//std::istream& operator>> (std::istream &s, Eigen::Vector3d &p);
//std::istream& operator>> (std::istream &s, Eigen::Matrix3d &m);
//std::istream& operator>> (std::istream &s, BDATA::Color &c);

#endif

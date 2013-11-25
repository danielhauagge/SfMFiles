// Copyright (C) 2011 by Daniel Hauagge
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


#ifndef __SFMF_BUNDLER_HPP__
#define __SFMF_BUNDLER_HPP__

#include <SfMFiles/sfmfiles>
#include <SfMFiles/FeatureDescriptors.hpp>

// Author: Daniel Hauagge <hauagge@cs.cornell.edu>
//   Date: 2011-04-03

// Reference for file format:
//     http://phototour.cs.washington.edu/bundler/bundler-v0.4-manual.html

// Coordinate conventions
//
// Cam: camera points towards -Z, Y points up, X points right
//  Im: Y points up, X points right, origin is lower left corner

BUNDLER_NAMESPACE_BEGIN

class Color
{
public:
    Color(uint8_t r_ = 0, uint8_t g_ = 0, uint8_t b_ = 0): r(r_), g(g_), b(b_) {}

    uint8_t r, g, b;
};

typedef struct {
    int pointIdx;
    int visibilityListIdx;
} PointVisListIdxs;

// Stores intrinsic and extrinsic camera parameters
class Camera
{
public:
    typedef std::vector<Camera> Vector;

    Camera();

    // Indexes of visible points, not stored in bundle file and only
    // computed if extra flag is passed to Reconstruction constructor or
    // function buildCam2PointIndex is called.
    std::vector<PointVisListIdxs> visiblePoints;

    // Extrinsic parameters
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;

    // Intrinsic parameters
    double focalLength; // Focal length
    double k1, k2; // Radial distortion parameters

    // Coordinate transforms
    void im2world(const Eigen::Vector2d &im, Eigen::Vector3d &w, int imWidth = 0, int imHeight = 0) const;
    void im2cam(const Eigen::Vector2d &im, Eigen::Vector3d &c, int imWidth = 0, int imHeight = 0) const;
    void cam2world(const Eigen::Vector3d &c, Eigen::Vector3d &w) const;
    void world2cam(const Eigen::Vector3d &w, Eigen::Vector3d &c) const;
    void world2cam(const Eigen::Vector4d &w, Eigen::Vector3d &c) const;

    /// @returns true if point lies inside image (if width or height were given) and is in front of camera
    bool world2im(const Eigen::Vector3d &w, Eigen::Vector2d &im, bool applyRadialDistortion = false, int imWidth = 0, int imHeight = 0) const;

    /// @returns true if point lies inside image (if width or height were given) and is in front of camera
    bool world2im(const Eigen::Vector4d &w, Eigen::Vector2d &im, bool applyRadialDistortion = false, int imWidth = 0, int imHeight = 0) const;

    /// @returns true if point lies inside image (if width or height were given) and is in front of camera
    bool cam2im(Eigen::Vector3d c, Eigen::Vector2d &im, bool applyRadialDistortion, int imWidth = 0, int imHeight = 0) const;

    // Outputs image coordinates that agree with PMVS (origin at upper left of image, x points right, y points down, pixel centered at (0.5, 0.5))
    void world2imPmvs(const Eigen::Vector3d &w, Eigen::Vector2d &im, bool applyRadialDistortion, int imWidth, int imHeight) const;
    void cam2imPmvs(Eigen::Vector3d c, Eigen::Vector2d &im, bool applyRadialDistortion, int imWidth, int imHeight) const;

    // Intrinsic matrix given image width and height
    void intrinsicMatrix(int imWidth, int imHeight, Eigen::Matrix3d &K) const;
    void invIntrinsicMatrix(int imWidth, int imHeight, Eigen::Matrix3d &invK) const;

    // Camera center in world coordinates
    void center(Eigen::Vector3d &centerVec) const;

    // Returns camera up vector in world coordinates
    void up(Eigen::Vector3d &upVec) const;

    // Returns direction in which camera is pointed at
    void lookingAt(Eigen::Vector3d &lat) const;

    // Did bundler sucessfully reconstruct this camera?
    bool isValid() const;
};

class ViewListEntry // formely PointEntry
{
public:
    typedef std::vector<ViewListEntry> Vector;

    int camera; /// Camera index
    int key; /// Keypoint index (into the .key file)
    Eigen::Vector2d keyPosition; // Keypoint position in image

    ViewListEntry();
    ViewListEntry(int camera, int key = -1, Eigen::Vector2d keyPosition = Eigen::Vector2d(0, 0));
    ViewListEntry(const ViewListEntry &other);
};

// Stores point location, color and list of cameras that can view this point
class Point // formerly PointEntry
{
public:
    typedef std::vector<Point> Vector;

    Eigen::Vector3d position;
    Color color;
    ViewListEntry::Vector viewList;
};

// Class that represents bundler output, encapsulating
// camera information and point information for reconstructed model.
class Reconstruction // formely BundlerData
{
public:
    static const char *ASCII_SIGNATURE;

    typedef boost::shared_ptr<Reconstruction> Ptr;
    static Reconstruction::Ptr New(const char *bundlerFileName, bool computeCam2PointIndex = false);

    Reconstruction(): _nValidCams(0), _cam2PointIndexInitialized(false) {};
    Reconstruction(const char *bundleFileName, const char *listFName, bool computeCam2PointIndex = false);
    Reconstruction(const char *bundleFileName, bool computeCam2PointIndex = false);
    Reconstruction(const Camera::Vector &cameras, const Point::Vector &points);

    void init(const char *bundleFileName, const char *listFName, bool computeCam2PointIndex = false);
    void init(const char *bundleFileName, bool computeCam2PointIndex = false);
    void init(const Camera::Vector &cameras, const Point::Vector &points);

    /// Load file with image filenames
    void readListFile(const char *listFName);
    void writeListFile(const char *listFName) const;
    bool listFileLoaded() const;

    const char *getListFileName() const;
    const char *getBundleFileName() const { return _bundleFName.c_str(); };

    /// Input/Output
    void readFile(const char *bundlerFileName, bool computeCam2PointIndex = false);
    void writeFile(const char *bundlerFileName) const;

    int getNCameras() const { return _cameras.size(); }
    int getNValidCameras() const { return _nValidCams; } // TODO: get rid of this
    int getNPoints() const { return _points.size(); }

    void buildCam2PointIndex();

    /// Returns size of image by looking at the header of the image file
    /// Assumes that the list file was loaded.
    /// @returns 0 for failure and non zero otherwise
    int getImageSizeForCamera(int camIdx, int &width, int &height, bool throwException = false) const;

    /// Loads sift features for a given camera. Uses image filename to
    /// generate the keypoint filename by replacing the image file extension
    /// with .key and .key.gz (tries to load both).
    /// @returns 0 for failure and non zero otherwise
    int loadSIFTFeaturesForCamera(int camIdx, std::vector<SIFTFeature> &sift, bool throwException = false) const;

    /// Accessors
    const Point::Vector &getPoints() const { return _points; };
    const Camera::Vector &getCameras() const { return _cameras; };
    const std::vector<std::string> &getImageFileNames() const { return _imageFNames; };
    Point::Vector &getPoints() { return _points; };
    Camera::Vector &getCameras() { return _cameras; };
    std::vector<std::string> &getImageFileNames() { return _imageFNames; };

protected:
    void _updateNValidCams(); // TODO: get rid of this

private:
    Camera::Vector _cameras;
    Point::Vector _points;
    int _nValidCams;  // TODO: get rid of this
    bool _cam2PointIndexInitialized;

    std::string _listFName, _bundleFName;
    std::vector<std::string> _imageFNames;
};

BUNDLER_NAMESPACE_END

std::istream &operator>> (std::istream &s, sfmf::Bundler::Camera &cam);

#endif // __SFMF_BUNDLER_HPP__

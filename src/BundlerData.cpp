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

#include "SfMFiles/BundlerData.hpp"
#include "utils.hpp"
#include "io.hpp"

#include <iomanip>

#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/device/file.hpp>

static
std::istream&
operator>>(std::istream& s, Eigen::Matrix3d& m)
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            s >> m(i, j);

    return s;
}

std::istream&
operator>>(std::istream& s, Eigen::Vector3d& p)
{
    for(int i = 0; i < 3; i++) s >> p[i];

    return s;
}

static inline
std::istream&
operator>>(std::istream& s, BDATA::Color& c)
{
    int aux;
    s >> aux;
    c.r = aux;
    s >> aux;
    c.g = aux;
    s >> aux;
    c.b = aux;

    return s;
}

static
std::istream&
operator>>(std::istream& s, Eigen::Vector2d& p)
{
    for(int i = 0; i < 2; i++) s >> p[i];

    return s;
}

std::istream&
operator>>(std::istream& s, BDATA::Camera& cam)
{
    s >> cam.focalLength >> cam.k1 >> cam.k2 >> cam.rotation >> cam.translation;
    return s;
}

std::ostream&
operator<<(std::ostream& s, const BDATA::Camera& cam)
{
    s << cam.focalLength << " " << cam.k1 << " " << cam.k2 << "\n";

    for(int i = 0; i < 3; i++) {
        const char* sep = "";
        for(int j = 0; j < 3; j++) {
            s << sep << cam.rotation(i, j);
            sep = " ";
        }
        s << "\n";
    }

    const char* sep = "";
    for(int i = 0; i < 3; i++) {
        s << sep << cam.translation(i);
        sep = " ";
    }
    s << "\n";

    return s;
}

BDATA::Camera::Camera()
{
    rotation.setIdentity();
    translation.setZero();
    focalLength = 1;
    k1 = k2 = 0;
}

bool
BDATA::Camera::isValid() const
{
    return fabs(rotation.determinant() - 1.0) < 0.00001;
}

void
BDATA::Camera::im2world(const Eigen::Vector2d& im, Eigen::Vector3d& w,
                        int imWidth, int imHeight) const
{
    Eigen::Vector3d c;

    // TODO: radial distortion
    im2cam(im, c, imWidth, imHeight);
    cam2world(c, w);
}

void
BDATA::Camera::im2cam(const Eigen::Vector2d& im, Eigen::Vector3d& c, int imWidth, int imHeight) const
{
    c[0] =  (  imWidth / 2.0 - im[0]) / focalLength;
    c[1] = -( imHeight / 2.0 - im[1]) / focalLength;
    c[2] = -1.0;
}

bool
BDATA::Camera::cam2im(Eigen::Vector3d c, Eigen::Vector2d& im,
                      bool applyRadialDistortion,
                      int imWidth, int imHeight) const
{
    applyRadialDistortion = applyRadialDistortion && (k1 != 0.0 || k2 != 0.0);

    bool isInFrontOfCamera = c[2] < 0.0;
    bool badZ = c[2] == 0.0; // point is at the center of projection
    c /= -c[2];

    if(applyRadialDistortion) {
        double cNorm2 = std::pow(c[0], 2.0) + std::pow(c[1], 2.0);
        double r = 1.0 + k1 * cNorm2 + k2 * std::pow(cNorm2, 2.0);

        c[0] *= r;
        c[1] *= r;
    }

    im[0] = imWidth / 2.0 + c[0] * focalLength;
    im[1] = imHeight / 2.0 + c[1] * focalLength;

    bool isInsideImage = true;
    if(imWidth > 0 && imHeight > 0) {
        isInsideImage = (im[0] >= 0) && (im[0] < imWidth) && (im[1] >= 0) && (im[1] < imHeight);
    }

    return isInsideImage && isInFrontOfCamera && !badZ;
}

bool
BDATA::Camera::world2im(const Eigen::Vector3d& w, Eigen::Vector2d& im,
                        bool applyRadialDistortion,
                        int imWidth, int imHeight) const
{
    Eigen::Vector3d c;
    world2cam(w, c);
    return cam2im(c, im, applyRadialDistortion, imWidth, imHeight);
}

bool
BDATA::Camera::world2im(const Eigen::Vector4d& w, Eigen::Vector2d& im,
                        bool applyRadialDistortion,
                        int imWidth, int imHeight) const
{
    Eigen::Vector3d c;
    world2cam(w, c);
    return cam2im(c, im, applyRadialDistortion, imWidth, imHeight);
}

void
BDATA::Camera::cam2imPmvs(Eigen::Vector3d c, Eigen::Vector2d& im,
                          bool applyRadialDistortion,
                          int imWidth, int imHeight) const
{
    cam2im(c, im, applyRadialDistortion, imWidth, imHeight);
    im[1] = imHeight - im[1] - 0.5;
    im[0] -= 0.5;
}

void
BDATA::Camera::world2imPmvs(const Eigen::Vector3d& w, Eigen::Vector2d& im, bool applyRadialDistortion, int imWidth, int imHeight) const
{
    Eigen::Vector3d c;
    world2cam(w, c);
    cam2imPmvs(c, im, applyRadialDistortion, imWidth, imHeight);
}

void
BDATA::Camera::cam2world(const Eigen::Vector3d& c, Eigen::Vector3d& w) const
{
    w = rotation.transpose() * (c - translation);
}

void
BDATA::Camera::world2cam(const Eigen::Vector3d& w, Eigen::Vector3d& c) const
{
    c = rotation * w + translation;
}

void
BDATA::Camera::world2cam(const Eigen::Vector4d& w, Eigen::Vector3d& c) const
{
    c = rotation * w.block(0, 0, 3, 1) + translation * w(3);
}

void
BDATA::Camera::intrinsicMatrix(int imWidth, int imHeight, Eigen::Matrix3d& K) const
{
    K.setZero();

    // z: points to the back of the camera
    // y: points up (opposite wrt image's y)
    // x: points to the right (agree's with images's x)
    // (0, 0) image -> (-x, y, -1) in camera
    K(0, 0) =  focalLength;
    K(1, 1) = -focalLength;
    K(0, 2) = -imWidth  / 2.0;
    K(1, 2) = -imHeight / 2.0;
    K(2, 2) = -1;
}

void
BDATA::Camera::invIntrinsicMatrix(int imWidth, int imHeight, Eigen::Matrix3d& invK) const
{
    invK.setZero();

    // z: points to the back of the camera
    // y: points up (opposite wrt image's y)
    // x: points to the right (agree's with images's x)
    // (0, 0) image -> (-x, y, -1) in camera
    invK(0, 0) =  1.0 / focalLength;
    invK(1, 1) = -1.0 / focalLength;
    invK(0, 2) = -imWidth  / (focalLength * 2.0);
    invK(1, 2) =  imHeight / (focalLength * 2.0);
    invK(2, 2) = -1.0;
}

void
BDATA::Camera::center(Eigen::Vector3d& centerW) const
{
    cam2world(Eigen::Vector3d::Zero(), centerW);
}

void
BDATA::Camera::up(Eigen::Vector3d& upW) const
{
    Eigen::Vector3d upCam(0, 1, 0);
    cam2world(upCam, upW);
    Eigen::Vector3d centerW;
    center(centerW);
    upW -= centerW;
    upW /= upW.norm();
}

void
BDATA::Camera::lookingAt(Eigen::Vector3d& lat) const
{
    Eigen::Vector3d imCenterC(0, 0, -1), imCenterW, camCenterW;
    cam2world(imCenterC, imCenterW);
    center(camCenterW);

    lat = (imCenterW - camCenterW);
    lat /= lat.norm();
}

BDATA::PointEntry::PointEntry(int camera_, int key_, Eigen::Vector2d keyPosition_):
    camera(camera_), key(key_), keyPosition(keyPosition_)
{
}

BDATA::PointEntry::PointEntry():
    camera(-1), key(-1)
{
}

BDATA::PointEntry::PointEntry(const BDATA::PointEntry& other)
{
    this->camera = other.camera;
    this->key = other.key;
    this->keyPosition = other.keyPosition;
}

const char* BDATA::BundlerData::ASCII_SIGNATURE = "# Bundle file v";

void
BDATA::BundlerData::_readFileASCII(boost::iostreams::filtering_istream& in)
{
    // File signature
    char sig[200];
    in.read(sig, strlen(ASCII_SIGNATURE));
    sig[strlen(ASCII_SIGNATURE)] = '\0';
    if (strcmp(sig, ASCII_SIGNATURE) != 0) {
        LOG_WARN("Bad signature in ASCII file: " << sig);
        throw sfmf::Error("Bad signature in binary file");
        return;
    }

    //char firstLine[255];
    //fgets(firstLine, sizeof(firstLine), file);

    double version;
    in >> version;
    if (version != 0.3) {
        std::stringstream err;
        err << "Unsupported version " << version;
        LOG_WARN(err.str());
        throw sfmf::Error(err.str());
        return;
    }

    // Get the number of points and cameras
    int nCameras, nPoints;
    in >> nCameras >> nPoints;

    // Read the cameras

    _cameras.resize(nCameras);
    PROGBAR_START("Read cameras");
    for(int i = 0; i < nCameras; i++) {
        PROGBAR_UPDATE(i, nCameras);

        Camera& cam = _cameras[i];

        // Focal length and radial distortion
        in >> cam.focalLength >> cam.k1 >> cam.k2;

        // Rotation
        double* R = &cam.rotation(0, 0);
        in >> cam.rotation(0, 0) >> cam.rotation(0, 1) >> cam.rotation(0, 2);
        in >> cam.rotation(1, 0) >> cam.rotation(1, 1) >> cam.rotation(1, 2);
        in >> cam.rotation(2, 0) >> cam.rotation(2, 1) >> cam.rotation(2, 2);

        // Translation
        double* t = &cam.translation(0, 0);
        in >> t[0] >> t[1] >> t[2];
    }
    // Read the points
    _points.resize(nPoints);
    PointInfo::Vector::iterator itPoint = _points.begin();

    PROGBAR_START("Read points");
    for(int i = 0; i < nPoints; i++, itPoint++) {
        PROGBAR_UPDATE(i, nPoints);

        // Position
        double* pos = &itPoint->position[0];
        in >> pos[0] >> pos[1] >> pos[2];

        // Color
        float r, g, b;
        in >> r >> g >> b;
        itPoint->color.r = (unsigned char)r;
        itPoint->color.g = (unsigned char)g;
        itPoint->color.b = (unsigned char)b;

        // View list
        int viewListSize;
        in >> viewListSize;
        itPoint->viewList.resize(viewListSize);

        PointEntry::Vector::iterator itEntry = itPoint->viewList.begin();

        for(int j = 0; j < viewListSize; j++, itEntry++) {
            in >> itEntry->camera >> itEntry->key >> itEntry->keyPosition(0) >> itEntry->keyPosition(1);
            assert(itEntry->camera < nCameras && itEntry->camera >= 0);
        }
    }
}

void
BDATA::BundlerData::readFile(const char* bundlerFileName, bool computeCamIndex)
{
    FILE* file = fopen(bundlerFileName, "rb");
    if(file == NULL) {
        std::stringstream errMsg;
        errMsg << "Could not open " << bundlerFileName << " for reading";
        throw sfmf::Error(errMsg.str().c_str());
    }

    // Read the first two bites
    char fileType[3];
    fread(fileType, sizeof(char), sizeof(fileType) - 1, file);
    fileType[2] = '\0';
    fclose(file);

    boost::iostreams::filtering_istream in;

    // Is this a GZIPed file?
    if (strcmp(fileType, "\x1f\x8b") == 0) {
        in.push(boost::iostreams::gzip_decompressor());
    }

    in.push(boost::iostreams::file_source(bundlerFileName));
    if(!in) {
        std::stringstream err;
        err << "Could not read file " << bundlerFileName;
        throw sfmf::Error(err.str());
        return;
    }

    _readFileASCII(in);

    _bundleFName = std::string(bundlerFileName);

    // Figure out how many of these cameras are actually valid
    _updateNValidCams();

    // Build camera to point index
    if(computeCamIndex) buildCam2PointIndex();
}

void
BDATA::BundlerData::_updateNValidCams()
{
    _nValidCams = 0;
    for(int i = 0; i < _cameras.size(); i++) {
        if(_cameras[i].isValid()) _nValidCams++;
    }
}

void
BDATA::BundlerData::init(const char* bundlerFileName, bool computeCamIndex)
{
    _cam2PointIndexInitialized = false;
    readFile(bundlerFileName, computeCamIndex);
}

void
BDATA::BundlerData::init(const char* bundlerFileName, const char* listFileName, bool computeCamIndex)
{
    _cam2PointIndexInitialized = false;
    init(bundlerFileName, computeCamIndex);
    readListFile(listFileName);
}

void
BDATA::BundlerData::init(const Camera::Vector& cameras, const PointInfo::Vector& points)
{
    _cam2PointIndexInitialized = false;
    _cameras = cameras;
    _points = points;
    _updateNValidCams();
}

void
BDATA::BundlerData::writeFile(const char* bundlerFileName) const
{
    std::ofstream f(bundlerFileName);
    f << std::scientific;
    f << std::setprecision(16);

    const int nCameras = getNCameras();
    const int nPoints = getNPoints();

    // Header
    f << "# Bundle file v0.3\n";

    // # of cameras and points
    f << nCameras << " " << getNPoints() << "\n";

    // Cameras
    PROGBAR_START("Writing cameras");
    for(int i = 0; i < nCameras; i++) {
        PROGBAR_UPDATE(i, nCameras);
        f << _cameras[i];
    }

    // Points
    PROGBAR_START("Writing cameras");
    for(int i = 0; i < nPoints; i++) {
        PROGBAR_UPDATE(i, nPoints);
        f << std::scientific
          << std::setprecision(16)
          << _points[i].position[0] << " "
          << _points[i].position[1] << " "
          << _points[i].position[2] << "\n"
          // Color
          << int(_points[i].color.r) << " "
          << int(_points[i].color.g) << " "
          << int(_points[i].color.b) << "\n"
          // View list
          << _points[i].viewList.size() << " ";

        // The view list
        for(int j = 0, jEnd = _points[i].viewList.size(); j < jEnd; j++) {
            f  << _points[i].viewList[j].camera << " "
               << _points[i].viewList[j].key << " "
               << _points[i].viewList[j].keyPosition(0) << " " << _points[i].viewList[j].keyPosition(1) << " ";
            assert(_points[i].viewList[j].camera >= 0 && _points[i].viewList[j].camera < nCameras);
        }
        f << "\n";
    }

    f.close();
}

void
BDATA::BundlerData::readListFile(const char* listFName)
{
    _imageFNames.resize(this->getNCameras());

    std::ifstream listF(listFName);
    int i = 0;
    for(i = 0; listF.good(); i++) {
        char line[1000];
        listF.getline(line, 1000);
        if(!listF.good()) break;

        if(i >= this->getNCameras()) {
            _imageFNames.resize(0);
            std::stringstream errMsg;
            errMsg << "Bad list file: number of filenames exceeds the number of cameras";
            throw sfmf::Error(errMsg.str());
        }

        std::istringstream lineStream(line);
        std::string fname;
        lineStream >> fname;
        _imageFNames[i] = fname;
    }

    if( i < this->getNCameras() ) {
        _imageFNames.resize(0);
        std::stringstream errMsg;
        errMsg << "Bad list file: number of filenames smaller than number of cameras (" << i << " vs " << this->getNCameras() << ")";
        throw sfmf::Error(errMsg.str());
    }

    _listFName = std::string(listFName);
}

void
BDATA::BundlerData::writeListFile(const char* listFName) const
{
    std::ofstream listF(listFName);
    if(!listF.good()) {
        std::stringstream errMsg;
        errMsg << "Could not open file " << listFName << " for writting";
        throw sfmf::Error(errMsg.str());
    }

    for(std::vector<std::string>::const_iterator fname = _imageFNames.begin(); fname != _imageFNames.end(); fname++ ) {
        listF << (*fname) << "\n";
    }
}

bool
BDATA::BundlerData::listFileLoaded() const
{
    return _imageFNames.size() != 0;
}

BDATA::BundlerData::BundlerData(const char* bundlerFileName, const char* listFileName, bool computeCam2PointIndex):
    _nValidCams(0)
{
    init(bundlerFileName, listFileName, computeCam2PointIndex);
}

BDATA::BundlerData::BundlerData(const char* bundlerFileName, bool computeCam2PointIndex):
    _nValidCams(0)
{
    init(bundlerFileName, computeCam2PointIndex);
}

BDATA::BundlerData::BundlerData(const Camera::Vector& cameras, const PointInfo::Vector& points):
    _nValidCams(0)
{
    init(cameras, points);
}

const char*
BDATA::BundlerData::getListFileName() const
{
    if(_listFName.size() ) return _listFName.c_str();
    return NULL;
}

BDATA::BundlerData::Ptr
BDATA::BundlerData::New(const char* bundleFileName, bool computeCam2PointIndex)
{
    BDATA::BundlerData::Ptr result;
    try {
        result = BundlerData::Ptr(new BundlerData(bundleFileName, computeCam2PointIndex));
    } catch (sfmf::Error e) {
        LOG_WARN("Caught exception");
        LOG_WARN("What: " << e.what());
        result = BDATA::BundlerData::Ptr();
    }
    return result;
}

void
BDATA::BundlerData::buildCam2PointIndex()
{
    if(_cam2PointIndexInitialized) return;

    int pntIdx = 0;
    for(PointInfo::Vector::iterator pnt = _points.begin(), pntEnd = _points.end(); pnt != pntEnd; pnt++, pntIdx++) {
        int peIdx = 0;
        for(PointEntry::Vector::iterator pe = pnt->viewList.begin(), peEnd = pnt->viewList.end(); pe != peEnd; pe++, peIdx++) {

            BDATA::PointVisListIdxs entry;
            entry.visibilityListIdx = peIdx;
            entry.pointIdx = pntIdx;

            _cameras[pe->camera].visiblePoints.push_back(entry);
        }
    }

    _cam2PointIndexInitialized = true;
}

int
BDATA::BundlerData::getImageSizeForCamera(int camIdx, int& width, int& height, bool throwException) const
{
    assert(listFileLoaded());
    return getImageSize(_imageFNames[camIdx].c_str(), width, height, throwException);
}

int
BDATA::BundlerData::loadSIFTFeaturesForCamera(int camIdx, std::vector<SIFTFeature>& sift, bool throwException) const
{
    std::string fname = _imageFNames[camIdx];
    int dotIdx = fname.find_last_of(".");

    std::string bname = fname.substr(0, dotIdx);

    const char* extensions[] = {".key", ".key.gz", NULL};
    bool done;
    for(int i = 0; (extensions[i] != NULL) && (!done); i++) {
        std::string keyFName = bname + extensions[i];
        done = loadSIFT(keyFName.c_str(), sift, false);
    }

    if(!done) {
        if(throwException) {
            std::stringstream errMsg;
            errMsg << "Could not find keypoint file for camera  " << camIdx;
            throw sfmf::IOError(errMsg.str());
        } else return 0;
    }

    return 1;
}

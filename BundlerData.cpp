#include "BundlerData.hpp"

//static inline
//std::istream& 
//operator>>(std::istream &s, Eigen::Matrix3d &m)
//{
//    for(int i = 0; i < 3; i++) 
//        for(int j = 0; j < 3; j++)
//            s >> m(i, j);
//    
//    return s;
//}
//
//static inline
//std::istream& 
//operator>>(std::istream &s, Eigen::Vector3d &p)
//{
//    for(int i = 0; i < 3; i++) s >> p[i];
//    
//    return s;
//}
//
//static inline
//std::istream& 
//operator>>(std::istream &s, BDATA::Color &c)
//{
//    int aux;
//    s >> aux; c.r = aux;
//    s >> aux; c.g = aux;
//    s >> aux; c.b = aux;
//    
//    return s;
//}
//
//static inline
//std::istream& 
//operator>>(std::istream &s, Eigen::Vector2d &p)
//{
//    for(int i = 0; i < 2; i++) s >> p[i];
//    
//    return s;
//}
//
//static inline
//std::istream& 
//operator>>(std::istream &s, BDATA::Camera &cam)
//{
//    s >> cam.focalLength >> cam.k1 >> cam.k2 >> cam.rotation >> cam.translation;
//    return s;
//}

std::ostream& 
operator<<(std::ostream &s, const BDATA::Camera &cam)
{
    s 
    << cam.focalLength << " " << cam.k1 << " " << cam.k2 << "\n";
    
    for(int i = 0; i < 3; i++) {
        const char *sep = "";
        for(int j = 0; j < 3; j++) {
            s << sep << cam.rotation(i,j);
            sep = " ";
        }
        s << "\n";
    }
    
    const char *sep = "";
    for(int i = 0; i < 3; i++) {
        s << sep << cam.translation(i);
        sep = " ";
    }
    s << "\n";
    
    return s;
}

void 
BDATA::Camera::cam2world(const Eigen::Vector3d &c, Eigen::Vector3d &w) const
{
    // TODO use Eigen methods to do this
    for(int i = 0; i < 3; i++) {
        w(i) = 0;
        for(int j = 0; j < 3; j++) {
            w(i) += rotation(j, i) * (c(j) - translation(j));
        }                
    }
}

void 
BDATA::Camera::world2cam(const Eigen::Vector3d &w, Eigen::Vector3d &c) const
{
    for(int i = 0; i < 3; i++) {
        c(i) = 0;
        for(int j = 0; j < 3; j++) {
            c(i) += rotation(i, j) * w(j) + translation(j);
        }                
    }   
}

BDATA::PointEntry::PointEntry(int camera_, int key_, Eigen::Vector2d keyPosition_): 
camera(camera_), key(key_), keyPosition(keyPosition_) 
{
}

BDATA::PointEntry::PointEntry(): 
camera(-1), key(-1) 
{
}

//void
//BDATA::BundlerData::init(const char *bundlerFileName)
//{
//#if 0
//    LOG("Old read from file");
//    std::ifstream f(bundlerFileName);
//#else
//    LOG("New read, buffers contente before parsing");
//    std::ifstream file(bundlerFileName);
//    
//    struct stat filestatus;
//    stat( bundlerFileName, &filestatus );
//    std::vector<char> buffer(filestatus.st_size);
//    file.read(&buffer[0], filestatus.st_size);
//    
//    std::stringstream f;
//    f.rdbuf()->pubsetbuf(&buffer[0], filestatus.st_size);    
//    //LOG("stopping before parsing data"); exit(1);
//#endif    
//    // Discard the first line
//    // TODO check the header to make sure this is a bundler file
//    f.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
//    
//    // Get the number of points and cameras
//    int nCameras, nPoints;
//    f >> nCameras >> nPoints;
//    
//    // Read the camaeras
//    _cameras.resize(nCameras);
//    for(int i = 0; i < nCameras; i++) f >> _cameras[i];
//    
//    // Read the points
//    _points.resize(nPoints);
//    for(int i = 0; i < nPoints; i++) {
//        f >> _points[i].position >> _points[i].color;
//        
//        // Read the view list
//        int viewListSize;
//        f >> viewListSize;
//        _points[i].viewList.resize(viewListSize);
//        for(int j = 0; j < viewListSize; j++) {
//            f >> _points[i].viewList[j].camera >> _points[i].viewList[j].key >> _points[i].viewList[j].keyPosition;
//        }
//    }
//    exit(1);
//}


void
BDATA::BundlerData::readASCII_v03(const char *data)
{
}

BDATA::BundlerData::readBin_v03(const char *data)
{
}

// This version uses C-style IO instead of C++ streams
void
BDATA::BundlerData::init(const char *bundlerFileName)
{
#if 0
    FILE* file = fopen(bundlerFileName, "r");
#else
    //LOG("New read, buffers contente before parsing");
    FILE* file = fopen(bundlerFileName, "r");
    
    struct stat filestatus;
    stat( bundlerFileName, &filestatus );
    std::vector<char> buffer(filestatus.st_size);
    file.read(&buffer[0], filestatus.st_size);
    
    std::stringstream f;
    f.rdbuf()->pubsetbuf(&buffer[0], filestatus.st_size);    
    //LOG("stopping before parsing data"); exit(1);
#endif    
    // Read the first line
	    char first_line[256];
    fgets(first_line, sizeof(first_line), file);
    double version;
    sscanf(first_line, "# Bundle file v%lf", &version);
    assert(version == 0.3);
    
    // Get the number of points and cameras
    int nCameras, nPoints;
    fscanf(file, "%d %d", &nCameras, &nPoints);
    
    // Read the camaeras
    _cameras.resize(nCameras);
    for(int i = 0; i < nCameras; i++) {
        Camera &cam = _cameras[i];
        
        // Focal length and radial distortion
        fscanf(file, "%lf %lf %lf\n", &cam.focalLength, &cam.k1, &cam.k2);
        
        // Rotation
        double *R = &cam.rotation(0,0);
        fscanf(file, "%lf %lf %lf\n%lf %lf %lf\n%lf %lf %lf\n", 
               &cam.rotation(0,0), &cam.rotation(0,1), &cam.rotation(0,2), 
               &cam.rotation(1,0), &cam.rotation(1,1), &cam.rotation(1,2), 
               &cam.rotation(2,0), &cam.rotation(2,1), &cam.rotation(2,2));
        
        // Translation
        double *t = &cam.translation(0,0);
        fscanf(file, "%lf %lf %lf\n", t+0, t+1, t+2);
    }
    
    // Read the points
    _points.resize(nPoints);
    std::vector<PointInfo>::iterator itPoint = _points.begin();
    for(int i = 0; i < nPoints; i++, itPoint++) {
                
        // Position
        double *pos = &itPoint->position[0];
        fscanf(file, "%lf %lf %lf\n", pos + 0, pos + 1, pos + 2);
        
        //std::cout << "pos = [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]" << std::endl;
        
        // Color
        float r, g, b;
        fscanf(file, "%f %f %f\n", &r, &g, &b);
        itPoint->color.r = (unsigned char)r;
        itPoint->color.g = (unsigned char)g;
        itPoint->color.b = (unsigned char)b;
        
        // Read the view list
        int viewListSize;
        fscanf(file, "%d", &viewListSize);
        itPoint->viewList.resize(viewListSize);
        std::vector<PointEntry>::iterator itEntry = itPoint->viewList.begin();
        for(int j = 0; j < viewListSize; j++, itEntry++) {
            fscanf(file, "%d %d %lf %lf", 
                   &itEntry->camera, &itEntry->key, 
                   &itEntry->keyPosition(0),  &itEntry->keyPosition(1));
            assert(itEntry->camera < nCameras && itEntry->camera >= 0);
        }
    }    
}

// FIXME get rid of C++ stream operators, makes code go really slow
void 
BDATA::BundlerData::writeToFile(const char *bundlerFileName)
{
    std::ofstream f(bundlerFileName);
    
    const int nCameras = getNCameras();
    const int nPoints = getNPoints();
    
    // Header
    f << "# Bundle file v0.3\n";
    
    // # of cameras and points
    f << nCameras << " " << getNPoints() << "\n";
    
    // Cameras
    for(int i = 0; i < nCameras; i++) f << _cameras[i];
    
    // Points
    for(int i = 0; i < nPoints; i++) {
        f 
        // Position
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
            f 
            << _points[i].viewList[j].camera << " " 
            << _points[i].viewList[j].key << " "
            << _points[i].viewList[j].keyPosition(0) << " " << _points[i].viewList[j].keyPosition(1) << " ";
            assert(_points[i].viewList[j].camera >= 0 && _points[i].viewList[j].camera < nCameras);
        }
        f << "\n";
    }

    f.close();
}

BDATA::BundlerData::BundlerData(const char *bundlerFileName)
{
    init(bundlerFileName);
}

BDATA::BundlerData::Ptr
BDATA::BundlerData::New(const char *bundleFileName)
{
    return BundlerData::Ptr(new BundlerData(bundleFileName));
}

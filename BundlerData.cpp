#include "BundlerData.hpp"

#define PRINT_VAR(var) std::cout << #var << " = " << (var) << std::endl

std::istream& 
operator>>(std::istream &s, BDATA::Matrix3x3 &m)
{
    for(int i = 0; i < 3; i++) 
        for(int j = 0; j < 3; j++)
            s >> m[i][j];
    
    return s;
}

std::istream& 
operator>>(std::istream &s, BDATA::Point3 &p)
{
    for(int i = 0; i < 3; i++) s >> p[i];
    
    return s;
}

std::istream& 
operator>>(std::istream &s, BDATA::Color &c)
{
    for(int i = 0; i < 3; i++) {
        int aux;
        s >> aux;
        c[i] = aux;
    }
    
    return s;
}

std::istream& 
operator>>(std::istream &s, BDATA::Point2 &p)
{
    for(int i = 0; i < 2; i++) s >> p[i];
    
    return s;
}

std::istream& 
operator>>(std::istream &s, BDATA::Camera &cam)
{
    s >> cam.focalLength >> cam.k1 >> cam.k2 >> cam.rotation >> cam.translation;
    return s;
}

void 
BDATA::Camera::cam2world(const Point3 &c, Point3 &w) const
{
    for(int i = 0; i < 3; i++) {
        w[i] = 0;
        for(int j = 0; j < 3; j++) {
            w[i] += rotation[j][i] * (c[j]  - translation[j]);
        }                
    }
}

void 
BDATA::Camera::world2cam(const Point3 &w, Point3 &c) const
{
    for(int i = 0; i < 3; i++) {
        c[i] = 0;
        for(int j = 0; j < 3; j++) {
            c[i] += rotation[i][j] * w[j] + translation[j];
        }                
    }   
}

void
BDATA::BundlerData::init(const char *bundlerFileName)
{
    std::ifstream f(bundlerFileName);
    
    // Discard the first line
    f.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    // Get the number of points and cameras
    int nCameras, nPoints;
    f >> nCameras >> nPoints;
    
    // Read the camaeras
    _cameras.resize(nCameras);
    for(int i = 0; i < nCameras; i++) {
        f >> _cameras[i];
        
        //PRINT_VAR(_cameras[i].translation[2]);
    }    
    //exit(1);
    
    // Read the points
    _points.resize(nPoints);
    
    for(int i = 0; i < nPoints; i++) {
        f >> _points[i].position >> _points[i].color;
        int viewListSize;
        f >> viewListSize;

#if 0
        PRINT_VAR(_points[i].position[0]);
        PRINT_VAR(_points[i].position[1]);
        PRINT_VAR(_points[i].position[2]);

        PRINT_VAR((int)_points[i].color[0]);
        PRINT_VAR((int)_points[i].color[1]);
        PRINT_VAR((int)_points[i].color[2]);
#endif
        
        // Read the view list
        _points[i].viewList.resize(viewListSize);
        for(int j = 0; j < viewListSize; j++) {
            f >> _points[i].viewList[j].camera >> _points[i].viewList[j].key >> _points[i].viewList[j].keyPosition;
        }
    }
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

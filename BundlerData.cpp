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

#include "BundlerData.hpp"

#include <Macros.hpp>

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

bool 
BDATA::Camera::isValid() const
{
  return fabs(rotation.determinant() - 1.0) < 0.00001;
}

void 
BDATA::Camera::im2world(int imWidth, int imHeight,
			const Eigen::Vector2d &im, Eigen::Vector3d &w) const
{
  Eigen::Vector3d c;
  c[0] = ( im[0] - imWidth/2.0)/focalLength;
  c[1] = (-im[1] + imHeight/2.0)/focalLength;
  c[2] = -1;
  cam2world(c, w);
}

void 
BDATA::Camera::world2im(int imWidth, int imHeight,
			const Eigen::Vector3d &w, Eigen::Vector2d &im) const
{
  Eigen::Vector3d c;
  world2cam(w, c);

  c[0] /= c[2];
  c[1] /= c[2];
  c[2] = 1;

  im[0] = imWidth/2.0 - focalLength * c[0];
  im[1] = imHeight/2.0 + focalLength * c[1];
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

void
BDATA::Camera::intrinsicMatrix(int imWidth, int imHeight, Eigen::Matrix3d &K) const
{
  K.setZero();

  // z: points to the back of the camera
  // y: points up (opposite wrt image's y)
  // x: points to the right (agree's with images's x)
  // (0, 0) image -> (-x, y, -1) in camera     
  K(0,0) =  focalLength;  
  K(1,1) = -focalLength;  
  K(0,2) = -imWidth  / 2.0;
  K(1,2) = -imHeight / 2.0;
  K(2,2) = -1;  
}

void
BDATA::Camera::invIntrinsicMatrix(int imWidth, int imHeight, Eigen::Matrix3d &invK) const
{
  invK.setZero();

  // z: points to the back of the camera
  // y: points up (opposite wrt image's y)
  // x: points to the right (agree's with images's x)
  // (0, 0) image -> (-x, y, -1) in camera     
  invK(0,0) =  1.0/focalLength;  
  invK(1,1) = -1.0/focalLength;  
  invK(0,2) = -imWidth  / (focalLength * 2.0);
  invK(1,2) =  imHeight / (focalLength * 2.0);
  invK(2,2) = -1;  
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

const char * BDATA::BundlerData::BINARY_SIGNATURE = "#bBundle file v";
const char * BDATA::BundlerData::ASCII_SIGNATURE = "# Bundle file v";

void
BDATA::BundlerData::readFileASCII(const char *bundlerFileName)
{
    FILE* file = fopen(bundlerFileName, "r");
    if(!file) {
      std::stringstream err;
      err << "Could not read file " << bundlerFileName;
      throw BadFileException(err.str());
      return;
    }

    // File signature
    char sig[200];
    size_t nread = fread(sig, sizeof(char), strlen(ASCII_SIGNATURE), file);
    sig[nread + 1] = '\0';
    if (strcmp(sig, ASCII_SIGNATURE) != 0) {
      LOG("ERROR: Bad signature in ASCII file: " << sig);
      throw BadFileException("Bad signature in binary file");
      return;
    }

    //char firstLine[255];
    //fgets(firstLine, sizeof(firstLine), file);
    
    double version;
    fscanf(file, "%lf", &version);
    if (version != 0.3) {
      std::stringstream err;
      err << "Unsupported version " << version; 
      LOG("ERROR: " << err.str());
      throw BadFileException(err.str());
      return;
    }

    // Get the number of points and cameras
    int nCameras, nPoints;
    fscanf(file, "%d %d", &nCameras, &nPoints);
    LOG(__PRETTY_FUNCTION__ << ":" << __LINE__);

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
    
    LOG(__PRETTY_FUNCTION__ << ":" << __LINE__);


    // Read the points
    _points.resize(nPoints);
    PointInfo::Vector::iterator itPoint = _points.begin();
    for(int i = 0; i < nPoints; i++, itPoint++) {
        
        // Position
        double *pos = &itPoint->position[0];
        fscanf(file, "%lf %lf %lf\n", pos + 0, pos + 1, pos + 2);
                
        // Color
        float r, g, b;
        fscanf(file, "%f %f %f\n", &r, &g, &b);
        itPoint->color.r = (unsigned char)r;
        itPoint->color.g = (unsigned char)g;
        itPoint->color.b = (unsigned char)b;
        
        // View list
        int viewListSize;
        fscanf(file, "%d", &viewListSize);
        itPoint->viewList.resize(viewListSize);

        PointEntry::Vector::iterator itEntry = itPoint->viewList.begin();

        for(int j = 0; j < viewListSize; j++, itEntry++) {
            fscanf(file, "%d %d %lf %lf", 
                   &itEntry->camera, &itEntry->key, 
                   &itEntry->keyPosition(0),  &itEntry->keyPosition(1));
            assert(itEntry->camera < nCameras && itEntry->camera >= 0);
        }
    }    


    fclose(file);
}

void
BDATA::BundlerData::readFileBinary(const char *bundlerFileName)
{
    FILE* file = fopen(bundlerFileName, "r");
    if(!file) {
      std::stringstream err;
      err << "Could not read file " << bundlerFileName;
      throw BadFileException(err.str());
      return;
    }
    
    // File signature
    char sig[200];
    fread(sig, sizeof(char), strlen(BINARY_SIGNATURE), file);
    sig[strlen(BINARY_SIGNATURE) + 1] = '\0';
    if (strcmp(sig, BINARY_SIGNATURE) != 0) {
      throw BadFileException("Bad signature in binary file");
      return;
    }
    
    // Version
    double version;
    fread(&version, sizeof(double), 1, file);
    if (version != 0.3) {
      std::stringstream err;
      err << "Unsupported version " << version; 
      LOG("ERROR: " << err.str());
      throw BadFileException(err.str());
      return;
    }
    
    // Get the number of points and cameras
    int nCameras, nPoints;
    fread(&nCameras, sizeof(int), 1, file);
    fread(&nPoints, sizeof(int), 1, file);
    
    // Read the camaeras
    _cameras.resize(nCameras);
    for(int i = 0; i < nCameras; i++) {
        Camera &cam = _cameras[i];
        
        // Focal length and radial distortion
        fread(&cam.focalLength, sizeof(double), 1, file);
        fread(&cam.k1, sizeof(double), 1, file);
        fread(&cam.k2, sizeof(double), 1, file);
        
        // Rotation
        double *R = &cam.rotation(0,0);
        fread(R, sizeof(double), 9, file);

        // Translation
        double *t = &cam.translation(0,0);
        fread(t, sizeof(double), 3, file);
    }
    
    // Read the points
    _points.resize(nPoints);
    PointInfo::Vector::iterator itPoint = _points.begin();
    for(int i = 0; i < nPoints; i++, itPoint++) {
        
        // Position
        double *pos = &itPoint->position[0];
        fread(pos, sizeof(double), 3, file);
        
        // Color
        unsigned char rgb[3];
        fread(rgb, sizeof(unsigned char), 3, file);
        itPoint->color.r = rgb[0];
        itPoint->color.g = rgb[1];
        itPoint->color.b = rgb[2];
        
        // Read the view list
        int viewListSize;
        fread(&viewListSize, sizeof(int), 1, file);
        itPoint->viewList.resize(viewListSize);
        PointEntry::Vector::iterator itEntry = itPoint->viewList.begin();
        for(int j = 0; j < viewListSize; j++, itEntry++) {
            fread(&itEntry->camera, sizeof(int), 1, file);
            fread(&itEntry->key, sizeof(int), 1, file);
            fread(&itEntry->keyPosition(0), sizeof(double), 2, file);
            assert(itEntry->camera < nCameras && itEntry->camera >= 0);
        }
    }
    
    fclose(file);
}

void
BDATA::BundlerData::readFile(const char *bundlerFileName)
{
    FILE* file = fopen(bundlerFileName, "rb");
    
    // Read the first two bites
    char fileType[3];
    fread(fileType, sizeof(char), sizeof(fileType) - 1, file);
    fileType[2] = '\0';
    fclose(file);
    if (strcmp(fileType, "#b") == 0) {
        readFileBinary(bundlerFileName);
    } else if(strcmp(fileType, "# ") == 0) {
        readFileASCII(bundlerFileName);        
    } else {
	throw BadFileException("File does not seem to be a bundle file.");
    }
}

void
BDATA::BundlerData::init(const char *bundlerFileName)
{
    readFile(bundlerFileName);
}

void 
BDATA::BundlerData::writeFile(const char *bundlerFileName, bool ASCII) const
{
    if (ASCII) writeFileASCII(bundlerFileName);
    else writeFileBinary(bundlerFileName);
}

// FIXME get rid of C++ stream operators, makes code go really slow
void 
BDATA::BundlerData::writeFileASCII(const char *bundlerFileName) const
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


// FIXME get rid of C++ stream operators, makes code go really slow
void 
BDATA::BundlerData::writeFileBinary(const char *bundlerFileName) const
{
    FILE* file = fopen(bundlerFileName, "wb");
    if(!file) {
        LOG("ERROR: Could not open file for writing " << bundlerFileName);
        return;
    }

    const int nCameras = getNCameras();
    const int nPoints = getNPoints();
    
    // Header
    fwrite(BINARY_SIGNATURE, sizeof(char), strlen(BINARY_SIGNATURE), file);
    double version= 0.3;
    fwrite(&version, sizeof(double), 1, file);
    
    // # of cameras and points
    fwrite(&nCameras, sizeof(int), 1, file);
    fwrite(&nPoints, sizeof(int), 1, file);
    
    // Cameras
    for(int i = 0; i < nCameras; i++) {
        const Camera &cam = _cameras[i];
        
        // Focal length and radial distortion
        fwrite(&cam.focalLength, sizeof(double), 1, file);
        fwrite(&cam.k1, sizeof(double), 1, file);
        fwrite(&cam.k2, sizeof(double), 1, file);
        
        // Rotation
        const double *R = &cam.rotation(0,0);
        fwrite(R, sizeof(double), 9, file);
        
        // Translation
        const double *t = &cam.translation(0,0);
        fwrite(t, sizeof(double), 3, file);
    }
    
    // Points
    PointInfo::Vector::const_iterator itPoint = _points.begin();
    for(int i = 0; i < nPoints; i++, itPoint++) {
        
        // Position
        const double *pos = &itPoint->position[0];
        fwrite(pos, sizeof(double), 3, file);
        
        // Color
        unsigned char rgb[3] = {itPoint->color.r, itPoint->color.g, itPoint->color.b};
        fwrite(rgb, sizeof(unsigned char), 3, file);
        
        // Write the view list
        int viewListSize = itPoint->viewList.size();
        fwrite(&viewListSize, sizeof(int), 1, file);
        PointEntry::Vector::const_iterator itEntry = itPoint->viewList.begin();
        for(int j = 0; j < viewListSize; j++, itEntry++) {
            fwrite(&itEntry->camera, sizeof(int), 1, file);
            fwrite(&itEntry->key, sizeof(int), 1, file);
            fwrite(&itEntry->keyPosition(0), sizeof(double), 2, file);
        }
    }
    
    fclose(file);
}


BDATA::BundlerData::BundlerData(const char *bundlerFileName)
{
    init(bundlerFileName);
}

BDATA::BundlerData::Ptr
BDATA::BundlerData::New(const char *bundleFileName)
{
  BDATA::BundlerData::Ptr result;
  try {
    result = BundlerData::Ptr(new BundlerData(bundleFileName));
  } catch (BadFileException e) {
    LOG("Caught exception");
    LOG("What: " << e.what());
    result = BDATA::BundlerData::Ptr();
  }
  return result;
}

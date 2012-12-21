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

#include "PMVSData.hpp"
#include <Macros.hpp>

// STD
#include <fstream>
#include <algorithm>

// Boost
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

static
void
readSeq(std::istream& s, std::vector<uint32_t>& seq)
{
    int len;
    s >> len;
    if(len > 0) {
        seq.resize(len);
        for(int i = 0; i < len; i++) {
            s >> seq[i];
        }
    } else if(len < 0) {
        int begin, end;
        s >> begin >> end;
        seq.resize(end - begin);
        for(int i = 0; i < seq.size(); i++) {
            seq[i] = begin + i;
        }
    }
}

std::istream& 
operator>>(std::istream& s, BDATA::PMVS::Options& opt)
{
    using namespace std;

    while(true && !s.eof()) {
        char firstToken[128];
        s.getline(firstToken, 128, ' ');
                
        if(strlen(firstToken) == 0) continue;
        if(firstToken[0] == '#') {
            s.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
             if(strcmp("level"      , firstToken) == 0) s >> opt.level;
        else if(strcmp("csize"      , firstToken) == 0) s >> opt.csize;
        else if(strcmp("threshold"  , firstToken) == 0) s >> opt.threshold;
        else if(strcmp("minImageNum", firstToken) == 0) s >> opt.minImageNum;
        else if(strcmp("setEdge"    , firstToken) == 0) s >> opt.setEdge;
        else if(strcmp("useBound"   , firstToken) == 0) s >> opt.useBound;
        else if(strcmp("useVisData" , firstToken) == 0) s >> opt.useVisData;
        else if(strcmp("sequence"   , firstToken) == 0) s >> opt.sequence;
        else if(strcmp("maxAngle"   , firstToken) == 0) s >> opt.maxAngle;
        else if(strcmp("quad"       , firstToken) == 0) s >> opt.quad;
        else if(strcmp("timages"    , firstToken) == 0) readSeq(s, opt.timages);
        else if(strcmp("oimages"    , firstToken) == 0) readSeq(s, opt.oimages);

        s.ignore(std::numeric_limits<std::streamsize>::max(), '\n');    
    }
        
    return s;
}

std::istream& 
operator>>(std::istream& s, BDATA::PMVS::Patch& p)
{
    assert(s.good());
    
    // Get patch type
    std::string patchType;
    s >> patchType;

    if(strcmp(patchType.c_str(), "PATCHS") == 0 || strcmp(patchType.c_str(), "PATCHPS") == 0) {
        s >> p.position(0) >> p.position(1) >> p.position(2) >> p.position(3);
        s >> p.normal(0)   >> p.normal(1)   >> p.normal(2)   >> p.normal(3);
        if(strcmp(patchType.c_str(), "PATCHPS") == 0) {
            s >> p.color(0) >> p.color(1) >> p.color(2);
        }
        s >> p.score >> p.debug1 >> p.debug2;        

        if(strcmp(patchType.c_str(), "PATCHPS") == 0) {
            s >> p.reconstructionAccuracy;
            s >> p.reconstructionSLevel;
        }
    } else {
        std::stringstream err;
        err << "Cannot handle patch of type " << patchType; 
        LOG("ERROR: " << err.str());
        throw BDATA::BadFileException(err.str());
    }

    int nGoodCameras = 0;
    s >> nGoodCameras;
    if(nGoodCameras > 0) {
        p.goodCameras.resize(nGoodCameras);
        for (int i = 0; i < nGoodCameras; i++) {
            s >> p.goodCameras[i];
        }
    }

    int nBadCameras = 0;
    s >> nBadCameras;
    if(nBadCameras > 0) {
        p.badCameras.resize(nBadCameras);
        for (int i = 0; i < nBadCameras; i++) {
            s >> p.badCameras[i];
        }
    }
    return s;
}

void 
BDATA::PMVS::Camera::world2im(const Eigen::Vector3d &w, Eigen::Vector2d &im) const
{
    Eigen::Vector4d wh(w[0], w[1], w[2], 1.0);
    Eigen::Vector3d imh = (*this) * wh;
    
    im[0] = imh[0] / imh[2];
    im[1] = imh[1] / imh[2];
}

BDATA::PMVS::PMVSData::PMVSData(const char *pmvsFileName, bool tryLoadOptionsFile)
{
    init(pmvsFileName, tryLoadOptionsFile);
}

void
BDATA::PMVS::PMVSData::init(const char *pmvsFileName, bool tryLoadOptionsFile)
{
    using namespace boost::filesystem;

    _maxNCameras = 0;
    _patchesFName = pmvsFileName;
    
    boost::progress_display* showProgress = NULL;
    
    std::ifstream f(pmvsFileName);

    // Fist line contains the string PATCHES
    f.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    unsigned int nPatches = 0;
    f >> nPatches;
    
    if(nPatches > 1000000) {
        showProgress = new boost::progress_display(nPatches, std::cout, "Loading patches:\n", "", "");
    }
    
    _patches.resize(nPatches);
    assert(_patches.size() == nPatches);
    for(unsigned int i = 0; i < nPatches; i++) {
        if(showProgress) ++(*showProgress);
        f >> _patches[i];
        
        for(int j = 0; j < _patches[i].goodCameras.size(); j++) {
            _maxNCameras = std::max(_patches[i].goodCameras[j], _maxNCameras);
        }
        for(int j = 0; j < _patches[i].badCameras.size(); j++) {
            _maxNCameras = std::max(_patches[i].badCameras[j], _maxNCameras);
        }
        
        _goodCamStats.accumulate(_patches[i].goodCameras.size());
        _badCamStats.accumulate(_patches[i].badCameras.size());
    }    
    _goodCamStats.finish();
    _badCamStats.finish();
    assert(_patches.size() == nPatches);

    // Load options file and fix camera indexes
    if(tryLoadOptionsFile) {
        path pathPatches(_patchesFName);

        std::string basename = pathPatches.leaf().string();
        std::string basenameNoExt(basename.begin(), basename.end() - strlen(".patch"));        

        path optionsPath(pathPatches.parent_path() / path("..") / basenameNoExt);

        LOG("Looking for options file " << optionsPath.string());
        if(exists(optionsPath)) {
            std::ifstream optF(optionsPath.string().c_str());
            BDATA::PMVS::Options opt;
            optF >> opt;

            if(opt.oimages.size() != 0) {
                throw sfmf::Error("Do not know how act when the size of oimages is not 0");
            }

            LOG("Remapping indexes");
            for(Patch::Vector::iterator p = _patches.begin(); p != _patches.end(); p++) {
                for(std::vector<uint32_t>::iterator cam = p->goodCameras.begin(); cam != p->goodCameras.end(); cam++) {
                    *cam = opt.timages[*cam];
                }
                // not entirelly sure about this part, maybe should use oimages here
                for(std::vector<uint32_t>::iterator cam = p->badCameras.begin(); cam != p->badCameras.end(); cam++) {
                    *cam = opt.timages[*cam];
                }
            }

        } else {
            LOG("Options file does not exist, moving along");
        }
    }
}

static
void
loadCamera(const char* fname, BDATA::PMVS::Camera& cam)
{
    std::ifstream camF(fname);
    std::string header;
    camF >> header;
    
    if(strcmp(header.c_str(), "CONTOUR") != 0) {
        std::stringstream errMsg;
        errMsg << "This does not seem to be a PMVS camera file\nFilename:" << fname << "\nHeader:" << header;
        throw std::runtime_error(errMsg.str());
    }
    
    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            camF >> cam(i,j);
        }
    }
}

void
BDATA::PMVS::PMVSData::loadCamerasAndImageFilenames()
{
    using namespace boost::filesystem;
    assert(_patchesFName.size());
    
    path pathPatches(_patchesFName);
    path camerasDir(pathPatches.parent_path() / path("../txt/"));
    path imagesDir(pathPatches.parent_path() / path("../visualize/"));
        
    if(!exists(camerasDir)) {
        LOG("Camera directory " << camerasDir << "does not seem to exist");
    } else {        

        std::set<uint32_t> allCams;
        for(Patch::Vector::iterator p = _patches.begin(); p != _patches.end(); p++) {
            for(std::vector<uint32_t>::iterator cam = p->goodCameras.begin(); cam != p->goodCameras.end(); cam++) {
                allCams.insert(*cam);
            }
            // not entirelly sure about this part, maybe should use oimages here
            for(std::vector<uint32_t>::iterator cam = p->badCameras.begin(); cam != p->badCameras.end(); cam++) {
                allCams.insert(*cam);
            }
        }

        boost::progress_display* showProgress = NULL;
        if(allCams.size() > 1000) {
            showProgress = new boost::progress_display(_maxNCameras, std::cout, "Loading cameras and images:\n", "", "");
        }

        for(std::set<uint32_t>::iterator cam = allCams.begin(); cam != allCams.end(); cam++) {
            if(showProgress) ++(*showProgress);

            char camFName[128];
            sprintf(camFName, "%08d.txt", *cam);
            path camPath = camerasDir / path(camFName);

            char imgFName[128];
            sprintf(imgFName, "%08d.jpg", *cam);
            path imgPath = imagesDir / path(imgFName);

            if(!exists(camPath)) {
                throw sfmf::Error("Could not load camera file");
            }
                        
            Camera P;
            loadCamera(camPath.c_str(), P);
            _cameras[*cam] = P;

            //if(exists(imgPath)) {
    	    _imageFNames[*cam] = imgPath.string();
	       //}
        }
    }
}

BDATA::PMVS::PMVSData::Ptr
BDATA::PMVS::PMVSData::New(const char *pmvsFileName, bool tryLoadOptionsFile)
{
    return PMVSData::Ptr(new PMVSData(pmvsFileName, tryLoadOptionsFile));
}

void 
BDATA::PMVS::PMVSData::printStats() const
{
    printf("# of cameras: %d\n", getNCameras());
    printf("# of patches: %d\n", getNPatches());
    
    printf("\nGood cameras:\n");
    printf("%10s: %5.0f\n", "Min", _goodCamStats.minVal);
    printf("%10s: %5.0f\n", "Max", _goodCamStats.maxVal);
    printf("%10s: %5.1f\n", "Mean", _goodCamStats.avgVal);
    printf("%10s: %5.0f\n", "Median", _goodCamStats.medianVal);

    printf("\nBad cameras:\n");
    printf("%10s: %5.0f\n", "Min", _badCamStats.minVal);
    printf("%10s: %5.0f\n", "Max", _badCamStats.maxVal);
    printf("%10s: %5.1f\n", "Mean", _badCamStats.avgVal);
    printf("%10s: %5.0f\n", "Median", _badCamStats.medianVal);
}

void 
BDATA::PMVS::PMVSData::Stats::accumulate(uint32_t sample)
{
    maxVal = std::max(maxVal, float(sample));
    minVal = std::min(minVal, float(sample));
    avgVal += sample;
    _samples.push_back(sample);
}

void 
BDATA::PMVS::PMVSData::Stats::finish()
{
    std::sort(_samples.begin(), _samples.end());
    medianVal = _samples[_samples.size()/2];
    avgVal /= _samples.size();
}






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

#include "SfMFiles/PMVS.hpp"
#include "io.hpp"

// STD
#include <fstream>
#include <algorithm>

// Boost
#include <boost/filesystem.hpp>

static
void
readSeq(std::istream &s, std::vector<uint32_t> &seq)
{
    int len;
    s >> len;
    if (len > 0) {
        seq.resize(len);
        for (int i = 0; i < len; i++) {
            s >> seq[i];
        }
    } else if (len < 0) {
        int begin, end;
        s >> begin >> end;
        seq.resize(end - begin);
        for (int i = 0; i < seq.size(); i++) {
            seq[i] = begin + i;
        }
    }
}

std::istream &
operator>>(std::istream &s, sfmf::PMVS::Options &opt)
{
    using namespace std;

    while (true && !s.eof()) {
        char firstToken[128];
        s.getline(firstToken, 128, ' ');

        if (strlen(firstToken) == 0) continue;
        if (firstToken[0] == '#') {
            s.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        if (strcmp("level"      , firstToken) == 0) s >> opt.level;
        else if (strcmp("csize"      , firstToken) == 0) s >> opt.csize;
        else if (strcmp("threshold"  , firstToken) == 0) s >> opt.threshold;
        else if (strcmp("minImageNum", firstToken) == 0) s >> opt.minImageNum;
        else if (strcmp("setEdge"    , firstToken) == 0) s >> opt.setEdge;
        else if (strcmp("useBound"   , firstToken) == 0) s >> opt.useBound;
        else if (strcmp("useVisData" , firstToken) == 0) s >> opt.useVisData;
        else if (strcmp("sequence"   , firstToken) == 0) s >> opt.sequence;
        else if (strcmp("maxAngle"   , firstToken) == 0) s >> opt.maxAngle;
        else if (strcmp("quad"       , firstToken) == 0) s >> opt.quad;
        else if (strcmp("timages"    , firstToken) == 0) readSeq(s, opt.timages);
        else if (strcmp("oimages"    , firstToken) == 0) readSeq(s, opt.oimages);

        s.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    return s;
}

std::istream &
operator>>(std::istream &s, sfmf::PMVS::Patch &p)
{
    assert(s.good());

    // Get patch type
    std::string patchType;
    s >> patchType;

    if (strcmp(patchType.c_str(), "PATCHS") == 0 || strcmp(patchType.c_str(), "PATCHPS") == 0) {
        s >> p.position(0) >> p.position(1) >> p.position(2) >> p.position(3);
        s >> p.normal(0)   >> p.normal(1)   >> p.normal(2)   >> p.normal(3);
        if (strcmp(patchType.c_str(), "PATCHPS") == 0) {
            s >> p.color(0) >> p.color(1) >> p.color(2);
        } else {
            p.color(0) = 0;
            p.color(1) = 0;
            p.color(2) = 0;
        }

        s >> p.score >> p.debug1 >> p.debug2;

        if (strcmp(patchType.c_str(), "PATCHPS") == 0) {
            s >> p.reconstructionAccuracy;
            s >> p.reconstructionSLevel;
        }
    } else {
        std::stringstream err;
        err << "Cannot handle patch of type " << patchType;
        LOG_WARN(err.str());
        throw sfmf::Error(err.str());
    }

    int nGoodCameras = 0;
    s >> nGoodCameras;
    if (nGoodCameras > 0) {
        p.goodCameras.resize(nGoodCameras);
        for (int i = 0; i < nGoodCameras; i++) {
            s >> p.goodCameras[i];
        }
    }

    int nBadCameras = 0;
    s >> nBadCameras;
    if (nBadCameras > 0) {
        p.badCameras.resize(nBadCameras);
        for (int i = 0; i < nBadCameras; i++) {
            s >> p.badCameras[i];
        }
    }
    return s;
}

std::ostream &
operator<<(std::ostream &s, const sfmf::PMVS::Patch &p)
{
    s << "PATCHPS\n";
    s << p.position(0) << " " << p.position(1) << " " << p.position(2) << " " << p.position(3) << "\n";
    s << p.normal(0) << " " << p.normal(1) << " " << p.normal(2) << " " << p.normal(3) << "\n";
    s << p.color(0) << " " << p.color(1) << " " << p.color(2) << "\n";
    s << p.score << " " << p.debug1 << " " << p.debug2 << "\n";
    s << p.reconstructionAccuracy << " " << p.reconstructionSLevel << "\n";

    s << p.goodCameras.size();
    for (int i = 0; i < p.goodCameras.size(); i++) s << " " << p.goodCameras[i];
    s << "\n";

    s << p.badCameras.size();
    for (int i = 0; i < p.badCameras.size(); i++) s << " " << p.badCameras[i];
    s << "\n";

    return s;
}

PMVS_NAMESPACE_BEGIN

void
Camera::world2im(const Eigen::Vector3d &w, Eigen::Vector2d &im) const
{
    Eigen::Vector4d wh(w[0], w[1], w[2], 1.0);
    Eigen::Vector3d imh = (*this) * wh;

    im[0] = imh[0] / imh[2];
    im[1] = imh[1] / imh[2];
}

bool
Camera::isValid() const
{
    return fabs(this->block<3, 3>(0, 0).determinant() - 1.0) < 0.00001;
}

Reconstruction::Reconstruction(const char *pmvsFileName, bool tryLoadOptionsFile)
{
    init(pmvsFileName, tryLoadOptionsFile);
}

void
Reconstruction::init(const char *pmvsFileName, bool tryLoadOptionsFile)
{
    using namespace boost::filesystem;

    _maxCamIdx = 0;
    _patchesFName = pmvsFileName;

    CompressedFileReader f(pmvsFileName);

    // Fist line contains the string PATCHES
    f.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    unsigned int nPatches = 0;
    f >> nPatches;

    LOG_INFO("PMVS file: " << pmvsFileName);
    PROGBAR_START("Loading in progress");

    _patches.resize(nPatches);
    assert(_patches.size() == nPatches);
    for (unsigned int i = 0; i < nPatches; i++) {
        PROGBAR_UPDATE(i, nPatches);
        f >> _patches[i];

        for (int j = 0; j < _patches[i].goodCameras.size(); j++) {
            _maxCamIdx = std::max(_patches[i].goodCameras[j], _maxCamIdx);
        }
        for (int j = 0; j < _patches[i].badCameras.size(); j++) {
            _maxCamIdx = std::max(_patches[i].badCameras[j], _maxCamIdx);
        }

        _goodCamStats.accumulate(_patches[i].goodCameras.size());
        _badCamStats.accumulate(_patches[i].badCameras.size());
    }

    _goodCamStats.finish();
    _badCamStats.finish();
    assert(_patches.size() == nPatches);

    // Load options file and fix camera indexes
    if (tryLoadOptionsFile) {
        path pathPatches(_patchesFName);

        std::string basename = pathPatches.leaf().string();
        std::string basenameNoExt(basename.begin(), basename.end() - strlen(".patch"));

        path optionsPath(pathPatches.parent_path() / path("..") / basenameNoExt);

        if (exists(optionsPath)) {
            LOG_INFO("Found options file " << optionsPath.string());
            CompressedFileReader optF(optionsPath.string().c_str());
            Options opt;
            optF >> opt;

            if (opt.oimages.size() != 0) {
                throw sfmf::Error("Do not know what to do when the size of oimages is not 0");
            }
#if 1
            LOG_INFO("Remapping indexes");
            for (Patch::Vector::iterator p = _patches.begin(); p != _patches.end(); p++) {
                for (std::vector<uint32_t>::iterator cam = p->goodCameras.begin(); cam != p->goodCameras.end(); cam++) {
                    *cam = opt.timages[*cam];
                }
                // not entirelly sure about this part, maybe should use oimages here
                for (std::vector<uint32_t>::iterator cam = p->badCameras.begin(); cam != p->badCameras.end(); cam++) {
                    *cam = opt.timages[*cam];
                }
            }
#endif

#if 1
        } else {
            LOG_INFO("Could not find options file");
        }
    }
#else
            _camIndexMapping = opt.timages;
        }
        else
        {
            PRINT_MSG("Options file does not exist, moving along");
            _camIndexMapping.resize(_maxCamIdx + 1);
            for (int i = 0; i < _maxCamIdx + 1; i++) _camIndexMapping[i] = i;
        }
#endif
}

static
void
loadCamera(const char *fname, Camera &cam)
{
    std::ifstream camF(fname);
    std::string header;
    camF >> header;

    if (strcmp(header.c_str(), "CONTOUR") != 0) {
        std::stringstream errMsg;
        errMsg << "This does not seem to be a PMVS camera file\nFilename:" << fname << "\nHeader:" << header;
        throw std::runtime_error(errMsg.str());
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            camF >> cam(i, j);
        }
    }
}

void
Reconstruction::loadCamerasAndImageFilenames(const char *basedir, bool loadOnlyUsedCameras)
{
    using namespace boost::filesystem;
    assert(_patchesFName.size());

    path basepath;
    if (strlen(basedir) == 0) {
        LOG_INFO("No basedir given, assuming that .patch file is within directory structure created by PMVS");
        path pathPatches(_patchesFName);
        basepath = pathPatches.parent_path() / path("../");
    } else {
        basepath = path(basedir);
    }

    path camerasDir(basepath / path("txt"));
    path imagesDir(basepath / path("visualize"));

    if (!exists(camerasDir)) {
        LOG_INFO("Camera directory " << camerasDir << " does not seem to exist");
    } else {
        std::set<uint32_t> allCams;

        if (loadOnlyUsedCameras) {
            for (Patch::Vector::iterator p = _patches.begin(); p != _patches.end(); p++) {
                for (std::vector<uint32_t>::iterator cam = p->goodCameras.begin(); cam != p->goodCameras.end(); cam++) {
                    allCams.insert(*cam);
                }
                // not entirelly sure about this part, maybe should use oimages here
                for (std::vector<uint32_t>::iterator cam = p->badCameras.begin(); cam != p->badCameras.end(); cam++) {
                    allCams.insert(*cam);
                }
            }
        } else {
            for (int i = 0; i < _maxCamIdx; i++) {
                allCams.insert(i);
            }
        }

        //_cameras.resize(_maxCamIdx + 1);
        //_imageFNames.resize(_maxCamIdx + 1);

        int camCount = 0;
        PROGBAR_START("Loading cameras and images");
        for (std::set<uint32_t>::iterator cam = allCams.begin(); cam != allCams.end(); cam++, camCount++) {
            PROGBAR_UPDATE(camCount, allCams.size());

            char camFName[128];
            //sprintf(camFName, "%08d.txt", _camIndexMapping[*cam]);
            sprintf(camFName, "%08d.txt", *cam);
            path camPath = camerasDir / path(camFName);

            char imgFName[128];
            //sprintf(imgFName, "%08d.jpg", _camIndexMapping[*cam]);
            sprintf(imgFName, "%08d.jpg", *cam);
            path imgPath = imagesDir / path(imgFName);

            if (!exists(camPath)) {
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

void
Reconstruction::writeFile(const char *patchesFileName) const
{
    std::ofstream patchesF(patchesFileName);

    patchesF << "PATCHES\n" << this->getNPatches() << "\n";

    for (Patch::Vector::const_iterator p = _patches.begin(); p != _patches.end(); p++) {
        patchesF << *p << "\n";
    }
}

void
Reconstruction::mergeWith(const Reconstruction &other)
{
    this->_patchesFName = "";
    std::copy(other._patches.begin(), other._patches.end(), std::back_inserter(this->_patches));
    this->_cameras.insert(other._cameras.begin(), other._cameras.end());
    this->_imageFNames.insert(other._imageFNames.begin(), other._imageFNames.end());
}

Reconstruction::Ptr
Reconstruction::New(const char *pmvsFileName, bool tryLoadOptionsFile)
{
    return Reconstruction::Ptr(new Reconstruction(pmvsFileName, tryLoadOptionsFile));
}

void
Reconstruction::printStats() const
{
    printf("# of cameras: %lu\n", getNCameras());
    printf("# of patches: %lu\n", getNPatches());

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
Reconstruction::Stats::accumulate(uint32_t sample)
{
    maxVal = std::max(maxVal, float(sample));
    minVal = std::min(minVal, float(sample));
    avgVal += sample;
    _samples.push_back(sample);
}

void
Reconstruction::Stats::finish()
{
    if (_samples.size() != 0) {
        std::sort(_samples.begin(), _samples.end());
        medianVal = _samples[_samples.size() / 2];
        avgVal /= _samples.size();
    }
}

PMVS_NAMESPACE_END
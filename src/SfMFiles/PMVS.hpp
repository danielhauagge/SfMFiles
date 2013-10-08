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


#ifndef __SFMF_PMVS_HPP__
#define __SFMF_PMVS_HPP__

#include <SfMFiles/sfmfiles>

// File format reference:
// http://grail.cs.washington.edu/software/pmvs/documentation.html

PMVS_NAMESPACE_BEGIN

/// Class that stores the information inside option files
class Options
{
public:
    std::vector<uint32_t> timages;
    std::vector<uint32_t> oimages;

    uint32_t level;
    uint32_t csize;
    float threshold;
    uint32_t wsize;
    uint32_t minImageNum;
    uint32_t CPU;
    uint32_t setEdge;
    bool useBound;
    bool useVisData;
    int sequence;
    float maxAngle;
    float quad;
};

class Camera: public Eigen::Matrix<double, 3, 4>
{
public:
    typedef std::vector<Camera> Vector;
    typedef std::map<uint32_t, Camera, std::less<int> > Map;

    // Coordinate transforms
    void world2im(const Eigen::Vector3d &w, Eigen::Vector2d &im) const;

    bool isValid() const;
};

class Patch
{
public:
    typedef std::vector<Patch> Vector;

    Eigen::Vector4d position, normal;
    double score; // Photometric consistency score, stays within -1 and 1 (good score)
    double debug1, debug2; // Numbers contained in the .patch file that are for debugging purposes
    std::vector<uint32_t> goodCameras; // Which cameras see this point?
    std::vector<uint32_t> badCameras;

    Eigen::Vector3f color;
    float reconstructionAccuracy;
    float reconstructionSLevel;

    Patch() {};
    Patch &operator = ( const Patch &source ) {
        assert(0);
        return *this;
    }

    Patch(const Patch &p) {
        this->score = p.score;
        this->debug1 = p.debug1;
        this->debug2 = p.debug2;
        this->position = p.position;
        this->normal = p.normal;
        this->goodCameras = p.goodCameras;
        this->badCameras = p.badCameras;
        this->color = p.color;
        this->reconstructionAccuracy = p.reconstructionAccuracy;
        this->reconstructionSLevel = p.reconstructionSLevel;
    }

    Patch(Patch &p) {
        this->score = p.score;
        this->debug1 = p.debug1;
        this->debug2 = p.debug2;
        this->position = p.position;
        this->normal = p.normal;
        this->goodCameras = p.goodCameras;
        this->badCameras = p.badCameras;
        this->color = p.color;
        this->reconstructionAccuracy = p.reconstructionAccuracy;
        this->reconstructionSLevel = p.reconstructionSLevel;
    }
};

/// Class that loads .patch files produced by PMVS
class Reconstruction
{
private:
    //std::vector<uint32_t> _camIndexMapping;

    Patch::Vector _patches;
    std::string _patchesFName; // Name of file containing patches data

    std::map<uint32_t, std::string> _imageFNames;

    Camera::Map _cameras;
    uint32_t _maxCamIdx; // Highest camera index that shows up in the patch file

    class Stats
    {
    private:
        // Used when computing the median
        std::vector<uint32_t> _samples;

    public:
        float maxVal, minVal;
        float avgVal, medianVal;

        Stats():
            maxVal(0.0), minVal(std::numeric_limits<float>::max()),
            avgVal(0.0), medianVal(0.0) {
        }

        void accumulate(uint32_t sample);
        void finish();
    };

    // Statistics about the file
    Stats _goodCamStats, _badCamStats; /// FIXME: remove this stuff, move into pmvs_info

public:
    typedef boost::shared_ptr<Reconstruction> Ptr;
    static Reconstruction::Ptr New(const char *pmvsFileName, bool tryLoadOptionsFile = true);

    Reconstruction() {};
    Reconstruction(const char *pmvsFileName, bool tryLoadOptionsFile = true);

    void init(const char *pmvsFileName, bool tryLoadOptionsFile = true);

    void writeFile(const char *patchesFileName) const;

    /// Loads cameras and image filenames. Expects .patch file to be inside the directory
    /// structure created by PMVS. That is, the .patch file should be within
    /// root/models/. The function will then try to read camera files
    /// from root/txt, it will also read the image filenames from
    /// root/visualize.
    void loadCamerasAndImageFilenames(const char *basedir = "", bool loadOnlyUsedCameras = true);

    size_t getNPatches() const {
        return _patches.size();
    }
    size_t getNCameras() const {
        return _cameras.size();
    }

    // Accessors
    const Patch::Vector &getPatches() const {
        return _patches;
    }
    Patch::Vector &getPatches() {
        return _patches;
    }

    const std::map<uint32_t, std::string> &getImageFileNames() const {
        return _imageFNames;
    };
    std::map<uint32_t, std::string> &getImageFileNames() {
        return _imageFNames;
    };

    const Camera::Map &getCameras() const {
        return _cameras;
    };
    Camera::Map &getCameras() {
        return _cameras;
    };

    void printStats() const;

    void mergeWith(const Reconstruction &other);
};

PMVS_NAMESPACE_END

#endif // __SFMF_PMVS_HPP__

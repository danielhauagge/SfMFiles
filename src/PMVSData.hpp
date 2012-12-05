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


#ifndef __PMVSDATA_HPP__
#define __PMVSDATA_HPP__

// std
#include <vector>
#include <map>
#include <iostream>

// Eigen3
#include <Eigen/Core>
#include <Eigen/LU>

// Boost libs
#include <boost/shared_ptr.hpp>

// File format reference: 
// http://grail.cs.washington.edu/software/pmvs/documentation.html

namespace BDATA
{    
    namespace PMVS
    {
        class Camera: public Eigen::Matrix<double, 3, 4>
        {
        public:
            typedef std::vector<Camera, Eigen::aligned_allocator<Camera> > Vector;      
            
            // Coordinate transforms
            void world2im(const Eigen::Vector3d &w, Eigen::Vector2d &im) const;
        };
        
        class Patch
        {
            //private:
            //Patch(Patch &p) {}
            
        public:            
            typedef std::vector<Patch, Eigen::aligned_allocator<Patch> > Vector;

            Eigen::Vector4d position, normal;
            double score; // Photometric consistency score, stays within -1 and 1 (good score)
            double debug1, debug2; // Numbers contained in the .patch file that are for debugging purposes
            std::vector<int> goodCameras; // To which cameras is this point visible?
            std::vector<int> badCameras;
            
            Eigen::Vector3f color;
            float reconstructionAccuracy;
            float reconstructionSLevel;
            
            Patch() {};
            Patch& operator = ( const Patch& source ) { assert(0); return *this;}
            
            Patch(const Patch &p) {
                this->score = p.score;
                this->debug1 = p.debug1;
                this->debug2 = p.debug2;
                this->position = p.position;
                this->normal = p.normal;
                this->goodCameras = p.goodCameras;
                this->badCameras = p.badCameras;
            }
            
            Patch(Patch &p) {
                this->score = p.score;
                this->debug1 = p.debug1;
                this->debug2 = p.debug2;
                this->position = p.position;
                this->normal = p.normal;
                this->goodCameras = p.goodCameras;
                this->badCameras = p.badCameras;
            }
            
        };
        
        //! Class that loads .patch files produced by PMVS
        class PMVSData
        {
        private:
            Patch::Vector _patches;
            std::string _patchesFName; // Name of file containing patches data
            
            std::vector<std::string> _imageFNames;
            
            Camera::Vector _cameras;
            
        public:
            typedef boost::shared_ptr<PMVSData> Ptr;
            static PMVSData::Ptr New(const char* pmvsFileName);
            
            PMVSData() {};
            PMVSData(const char* pmvsFileName);
            void init(const char* pmvsFileName);
                                
            //! Loads cameras and image filenames. Expects .patch file to be inside the directory
            //! structure created by PMVS. That is, the .patch file should be within
            //! root/models/. The function will then try to read camera files
            //! from root/txt, it will also read the image filenames from
            //! root/visualize.
            void loadCamerasAndImageFilenames();

            int getNPatches() const { return _patches.size(); }
            int getNCameras() const { return _cameras.size(); }
            
            // Accessors
            const Patch::Vector& getPatches() const { return _patches; }
            Patch::Vector& getPatches() { return _patches; }
            const std::vector<std::string>& getImageFileNames() const { return _imageFNames; };
            std::vector<std::string>& getImageFileNames() { return _imageFNames; };
            const Camera::Vector& getCameras() const { return _cameras; };
            Camera::Vector& getCameras() { return _cameras; };
        };
    }
}

#endif

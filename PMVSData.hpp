#ifndef __PMVSDATA_HPP__
#define __PMVSDATA_HPP__

// std
#include <vector>
#include <iostream>b

// Eigen3
#include <Eigen/Core>
#include <Eigen/LU>

// Boost libs
#include <boost/shared_ptr.hpp>

#include <Macros.hpp>

// File format reference: 
// http://grail.cs.washington.edu/software/pmvs/documentation.html

namespace BDATA
{
    namespace PMVS
    {
        class Patch
        {
        private:
            Patch(Patch &p) {}
            
        public:
            Eigen::Vector4d position, normal;
            double score; // Photometric consistency score, stays within -1 and 1 (good score)
            double debug1, debug2; // Numbers contained in the .patch file that are for debugging purposes
            std::vector<int> goodCameras; // To which cameras is this point visible?
            std::vector<int> badCameras;
                        
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
        };
        
        //! Class that loads .patch files produced by PMVS
        class PMVSData
        {
        private:
            std::vector<Patch> _patches;
            
        public:
            typedef boost::shared_ptr<PMVSData> Ptr;
            static PMVSData::Ptr New(const char *pmvsFileName);
            
            PMVSData() {};
            PMVSData(const char *pmvsFileName);
            void init(const char *pmvsFileName);
            
            int getNPatches() const { return _patches.size(); }
            
            const std::vector<Patch> &getPatches() const { return _patches; }
        };
    }
}

#endif
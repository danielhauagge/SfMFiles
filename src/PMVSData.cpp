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
#include <fstream>

std::istream& 
operator>>(std::istream &s, BDATA::PMVS::Patch &p)
{
    assert(s.good());
    
    // First line of every patch is "PATCHS"
    while(true) {
        char c;
        s.get(c);
        if(c == 'P' or c == 'p') break;
    }
    s.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    s >> p.position(0) >> p.position(1) >> p.position(2) >> p.position(3);
    s >> p.normal(0)   >> p.normal(1)   >> p.normal(2)   >> p.normal(3);
    s >> p.score >> p.debug1 >> p.debug2;

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


BDATA::PMVS::PMVSData::PMVSData(const char *pmvsFileName)
{
    init(pmvsFileName);
}

void
BDATA::PMVS::PMVSData::init(const char *pmvsFileName)
{
    std::ifstream f(pmvsFileName);

    // Fist line contains the string PATCHES
    f.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    unsigned int nPatches = 0;
    f >> nPatches;
    
    _patches.resize(nPatches);
    assert(_patches.size() == nPatches);
    for(unsigned int i = 0; i < nPatches; i++) {
        f >> _patches[i];
    }    
    assert(_patches.size() == nPatches);

#if 0
    LOG("Done reading patches");    
    PRINT_VAR(_patches[nPatches - 1].position);
    PRINT_VAR(_patches[nPatches - 1].normal);
    PRINT_VAR(_patches[nPatches - 1].score);
    PRINT_VAR(_patches[nPatches - 1].debug1);
    PRINT_VAR(_patches[nPatches - 1].debug2);
    PRINT_VAR(_patches[nPatches - 1].goodCameras.size());
    PRINT_VAR(_patches[nPatches - 1].badCameras.size());
#endif
}

BDATA::PMVS::PMVSData::Ptr
BDATA::PMVS::PMVSData::New(const char *pmvsFileName)
{
    return PMVSData::Ptr(new PMVSData(pmvsFileName));
}

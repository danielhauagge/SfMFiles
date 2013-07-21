// Copyright (C) 2013 by Daniel Hauagge
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

// Other projects
#include <SfMFiles/sfmfiles>
#include <OptParser/optparser>

// STD
#include <iostream>
#include <iomanip>

// Boost
#include <boost/filesystem.hpp>

std::string
getBasename(const std::string& fname)
{
    namespace fs = boost::filesystem;
    fs::path p(fname);
    return p.filename().string();
}

int
main(int argc, const char** argv)
{
    using namespace BDATA;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("[OPTIONS] <in:bundle.out> <in:list.txt> <out:bundle.out> <out:list.txt>");
    optParser.addDescription("Removes duplicated cameras from bundler reconstruction (based on image filename).");
    optParser.setNArguments(4, 4);
    optParser.parse(argc, argv);

    std::string inBundleFName  = args[0];
    std::string inListFName    = args[1];
    std::string outBundleFName = args[2];
    std::string outListFName   = args[3];

    BundlerData bundle(inBundleFName.c_str());
    bundle.readListFile(inListFName.c_str());

    // Find out which are the repeated images
    std::set<std::string> imageBNames; // Stores image basenames
    std::set<int> camIdxsToRemove; // indexes of the cameras that will be remomoved
    std::map<int, int> newCamMapping; // mapping old to new camera indexes
    std::vector<std::string> newImageList;
    Camera::Vector newCameras;
    for(int camIdx = 0; camIdx < bundle.getNCameras(); camIdx++) {
        std::string bname = getBasename(bundle.getImageFileNames()[camIdx]);

        if(imageBNames.count(bname) != 0) {
            camIdxsToRemove.insert(camIdx);
            continue;
        }

        imageBNames.insert(bname);
        newImageList.push_back(bundle.getImageFileNames()[camIdx]);
        newCameras.push_back(bundle.getCameras()[camIdx]);
        newCamMapping[camIdx] = newImageList.size() - 1;
    }

    PRINT_MSG("Removing " << camIdxsToRemove.size() << " of " << bundle.getNCameras() << " cameras");

    PointInfo::Vector newPoints;
    for(int pntIdx = 0; pntIdx < bundle.getNPoints(); pntIdx++) {
        const PointInfo& pntInfo = bundle.getPointInfo()[pntIdx];
        PointInfo newPntInfo(pntInfo);

        newPntInfo.viewList.resize(0);

        for(int i = 0; i < pntInfo.viewList.size(); i++) {
            if( camIdxsToRemove.count( pntInfo.viewList[i].camera ) ) continue;
            PointEntry pntEntry = pntInfo.viewList[i];
            pntEntry.camera = newCamMapping[pntEntry.camera];

            newPntInfo.viewList.push_back( pntEntry );
        }

        newPoints.push_back(newPntInfo);
    }

    BundlerData outBundle(newCameras, newPoints);
    outBundle.getImageFileNames() = newImageList;
    outBundle.writeFile(outBundleFName.c_str());
    outBundle.writeListFile(outListFName.c_str());

    return EXIT_SUCCESS;
}
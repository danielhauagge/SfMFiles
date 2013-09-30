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
using namespace sfmf;
#include <CMDCore/optparser>
#include <CMDCore/logger>

// STD
#include <iostream>
#include <iomanip>

// Boost
#include <boost/filesystem.hpp>

std::string
getBasename(const std::string &fname)
{
    namespace fs = boost::filesystem;
    fs::path p(fname);
    return p.filename().string();
}

int
main(int argc, const char **argv)
{
    using namespace Bundler;
    using namespace cmdc;
    using namespace std;

    Logger::setLogLevels(LOGLEVEL_DEBUG);

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("<in:bundle.out> <in:list.txt> <out:bundle.out> <out:list.txt>");
    optParser.addDescription("Removes duplicated cameras from bundler reconstruction (based on image filename), keeps camera that sees more points.");
    optParser.setNArguments(4, 4);
    optParser.parse(argc, argv);

    string inBundleFName  = args[0];
    string inListFName    = args[1];
    string outBundleFName = args[2];
    string outListFName   = args[3];

    // Load old reconstruction
    Reconstruction bundle(inBundleFName.c_str());
    bundle.readListFile(inListFName.c_str());
    bundle.buildCam2PointIndex();

    // Find out which are the repeated images
    map<string, int> selectedCams; // For each image basename what is the index in the original bundle that we should keep
    set<int> camIdxsToKeep;        // Indexes of the cameras that will be remomoved from viewlists
    map<int, int> newCamMapping;   // Mapping old to new camera indexes
    vector<string> newImageList;
    Camera::Vector newCameras;
    Camera::Vector &oldCameras = bundle.getCameras();
    Camera::Vector::iterator oldCam = oldCameras.begin();
    for(int camIdx = 0; camIdx < bundle.getNCameras(); camIdx++, oldCam++) {
        string bname = getBasename(bundle.getImageFileNames()[camIdx]);

        if(selectedCams.find(bname) != selectedCams.end()) {
            // Keep the camera that sees more points
            if(oldCam->visiblePoints.size() > oldCameras[selectedCams[bname]].visiblePoints.size()) {
                selectedCams[bname] = camIdx;
            }
            continue;
        }

        selectedCams[bname] = camIdx;
    }

    for(map<string, int>::iterator it = selectedCams.begin(); it != selectedCams.end(); it++) {
        newImageList.push_back(bundle.getImageFileNames()[it->second]);
        newCameras.push_back(oldCameras[it->second]);
        newCamMapping[it->second] = newImageList.size() - 1;
        camIdxsToKeep.insert(it->second);
    }

    LOG_INFO("Keeping " << camIdxsToKeep.size() << " of " << bundle.getNCameras() << " cameras");

    Point::Vector newPoints;
    for(int pntIdx = 0; pntIdx < bundle.getNPoints(); pntIdx++) {
        const Point &pntInfo = bundle.getPoints()[pntIdx];
        Point newPntInfo(pntInfo);

        newPntInfo.viewList.resize(0);

        for(int i = 0; i < pntInfo.viewList.size(); i++) {
            if( camIdxsToKeep.count( pntInfo.viewList[i].camera ) == 0) continue;
            ViewListEntry pntEntry = pntInfo.viewList[i];
            pntEntry.camera = newCamMapping[pntEntry.camera];

            newPntInfo.viewList.push_back( pntEntry );
        }

        newPoints.push_back(newPntInfo);
    }

    Reconstruction outBundle(newCameras, newPoints);
    outBundle.getImageFileNames() = newImageList;
    outBundle.writeFile(outBundleFName.c_str());
    outBundle.writeListFile(outListFName.c_str());

    return EXIT_SUCCESS;
}
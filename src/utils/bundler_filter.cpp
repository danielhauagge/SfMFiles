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

#include <SfMFiles/sfmfiles>
using namespace sfmf;

#include <CMDCore/optparser>

#include <algorithm>

void
filterByNCams(Bundler::Reconstruction &bundler, int minNCams)
{
    using namespace Bundler;

    Point::Vector newPnts;
    PROGBAR_START("Processing points");
    int idx = 0;
    int nPnts = bundler.getNPoints();
    int nCulled = 0;
    for(Point::Vector::iterator it = bundler.getPoints().begin(), itEnd = bundler.getPoints().end(); it != itEnd; it++, idx++) {
        PROGBAR_UPDATE(idx, nPnts);
        if(it->viewList.size() >= minNCams) {
            newPnts.push_back(*it);
        } else {
            nCulled++;
        }
    }

    LOG_INFO(nCulled << "/" << bundler.getNPoints() << " points were removed");

    bundler.getPoints() = newPnts;
}

void
removeCameras(const std::string &rmCamsFName, Bundler::Reconstruction &bundler)
{
    using namespace std;

    ifstream f(rmCamsFName.c_str());
    if(f.bad()) {
        LOG_ERROR("Could not open file " << rmCamsFName << " for reading");
        exit(EXIT_FAILURE);
    }

    vector<int> rmCamIdxs;
    while(f.good()) {
        int camIdx;
        f >> camIdx;
        rmCamIdxs.push_back(camIdx);
    }

    LOG_INFO(rmCamIdxs.size() << " indexes in file");
    sort(rmCamIdxs.begin(), rmCamIdxs.end());

    Bundler::Camera::Vector newCams;
    Bundler::Camera::Vector &cams = bundler.getCameras();

    vector<string> &imgFNames = bundler.getImageFileNames();
    vector<string> newImgFNames;

    vector<int>::iterator rmIdx = rmCamIdxs.end();
    int currIdx = 0;
    while(true) {
        for(; rmIdx != rmCamIdxs.end() && (*rmIdx) >= currIdx; rmIdx++);
        if(rmIdx == rmCamIdxs.end()) break;

        for(; currIdx < cams.size() && currIdx < (*rmIdx); currIdx++) {
            newCams.push_back(cams[currIdx]);

            if(bundler.listFileLoaded()) newImgFNames.push_back(imgFNames[currIdx]);
        }
        if(currIdx == cams.size()) break;
    }

    bundler.getCameras() = newCams;

    if(bundler.listFileLoaded()) {
        bundler.getImageFileNames() = newImgFNames;
    }
}

int
main(int argc, char const *argv[])
{
    using namespace Bundler;
    using namespace cmdc;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("<in:old.out> <out:new.out>");
    optParser.addDescription("Utility that provides multiple ways of filtering points from a bundler file.");
    // optParser.addOption("boundingSphere", "", "S", "--bounding-sphere",
    //                     "Keep all points that fall within a bounding sphere. Specify as X,Y,Z,R where "
    //                     "X, Y, and Z are center coordinates and R is the radius.");
    optParser.addOption("minNCams", "-n", "N", "--by-ncams",
                        "Keep all points that are seen by at least N cameras", "-1");
    optParser.addOption("rmCamsFName", "-r", "FNAME", "--rm-cams",
                        "Remove cameras listed in file FNAME (one camera per line, zero indexed)");
    optParser.addOption("inListFName", "", "FNAME", "--in-list", "Input list filename");
    optParser.addOption("outListFName", "", "FNAME", "--out-list", "Output list filename");
    optParser.parse(argc, argv);

    std::string inBundleFName = args[0];
    std::string outBundleFName = args[1];

    int minNCams = opts["minNCams"].asInt();
    std::string rmCamsFName = opts["rmCamsFName"];
    std::string inListFName = opts["inListFName"];
    std::string outListFName = opts["outListFName"];

    Reconstruction bundler(inBundleFName.c_str());
    if(inListFName.size()) bundler.readListFile(inListFName.c_str());

    if(rmCamsFName.size()) removeCameras(rmCamsFName, bundler);
    if(minNCams >= 0) filterByNCams(bundler, minNCams);

    LOG_INFO("Writing output to " << outBundleFName);
    bundler.writeFile(outBundleFName.c_str());

    if(outListFName.size()) {
        if(!bundler.listFileLoaded()) {
            LOG_ERROR("No list file loaded but output list filename specified");
            return EXIT_FAILURE;
        }

        bundler.writeListFile(outBundleFName.c_str());
    }

    return EXIT_SUCCESS;
}
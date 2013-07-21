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
basename(const std::string& fname)
{
    using namespace boost::filesystem;

    path pathFName(fname);
    std::string bname = pathFName.leaf().string();

    return bname;
}

int
main(int argc, char const* argv[])
{
    using namespace BDATA;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("[OPTIONS] <out:bundle.out> <out:list.txt> <in:bundle1.out> <in:list1.txt> <in:bundle2.out> <in:list2.txt> ...");
    optParser.addDescription("This command merges bundle files and update the visibility lists for bundle files that the same number of points");
    optParser.addFlag("dontUpdateVizList", "-d", "--no-viz-update",
                      "Do not update the point visibility lists (if bundle files have different number of point this must be enabled)");
    optParser.parse(argc, argv);

    std::string outBundleFName = args[0];
    std::string outListFName = args[1];

    bool updateVizList = !opts["dontUpdateVizList"].asBool();

    std::vector<std::string> inBundleFNames;
    std::vector<std::string> inListFNames;

    for (int i = 2; i < args.size(); i += 2) {
        inBundleFNames.push_back(args[i]);
        inListFNames.push_back(args[i + 1]);
    }

    PRINT_MSG("Merging " << inBundleFNames.size() << " bundle files");

    std::vector<std::string> outImageList;
    Camera::Vector outCams;
    PointInfo::Vector outPoints;

    // We don't add a camera more than once, this keeps track of which ones
    // we've seen.
    std::set<std::string> imgsSeen;

    for (int bun = 0, nPoints = -1; bun < inBundleFNames.size(); bun++) {
        PRINT_MSG("[" << std::setw(4) << bun << "/" << inBundleFNames.size() << "] Loading " << inBundleFNames[bun]);
        BundlerData bundle(inBundleFNames[bun].c_str());
        bundle.readListFile(inListFNames[bun].c_str());

        if (updateVizList) {
            PRINT_MSG("Building index camera -> visible points");
            bundle.buildCam2PointIndex();
        }

        if (bun == 0) {
            nPoints = bundle.getNPoints();
            outPoints = bundle.getPointInfo();
            outCams = bundle.getCameras();

            for (int i = 0; i < bundle.getNCameras(); i++) {
                imgsSeen.insert(basename(bundle.getImageFileNames()[i]));
                outImageList.push_back(bundle.getImageFileNames()[i]);
            }
            continue;
        }

        // Make sure we're not trying to merge bundle files that have different points
        if (nPoints != bundle.getNPoints() && updateVizList) {
            PRINT_MSG("Seems like not all bundle files have the same number of points.\n>> You should only merge bundle reconstruction that have the same points but different cameras.>>I'm out!!");
            return EXIT_FAILURE;
        }

        std::vector<std::string>& imageList = bundle.getImageFileNames();
        const Camera::Vector& cams = bundle.getCameras();

        int nAdded = 0;
        for (int imgIdx = 0; imgIdx < bundle.getNCameras(); imgIdx++) {
            std::string imgFName = imageList[imgIdx];

            // Make sure we're not trying to insert something we've seen already
            std::string imgBName = basename(imgFName);
            if (imgsSeen.count(imgBName) != 0) {
                continue;
            }
            imgsSeen.insert(imgBName);

            nAdded++;

            // Add the new camera
            outCams.push_back(cams[imgIdx]);
            outImageList.push_back(imageList[imgIdx]);
            int newCamIdx = outCams.size() - 1;

            // Udate the visibility lists
            if (updateVizList) {
                for (std::vector<int>::const_iterator pntIdx = cams[imgIdx].visiblePoints.begin(); pntIdx != cams[imgIdx].visiblePoints.end(); pntIdx++) {
                    outPoints[*pntIdx].viewList.push_back(PointEntry(newCamIdx));
                }
            }
        }
        PRINT_MSG(std::setw(8) << nAdded << "/" << bundle.getNCameras() << " cameras were added");

    }

    PRINT_MSG("Writing bundle file to " << outBundleFName);
    BundlerData outBundle;
    outBundle.getCameras() = outCams;
    outBundle.getPointInfo() = outPoints;
    outBundle.writeFile(outBundleFName.c_str());

    PRINT_MSG("Writing image list to " << outListFName);
    std::ofstream outListF(outListFName.c_str());
    for (int i = 0; i < outImageList.size(); i++) {
        outListF << outImageList[i] << std::endl;
    }

    return EXIT_SUCCESS;
}

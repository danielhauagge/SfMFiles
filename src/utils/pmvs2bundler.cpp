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

#include <iostream>
#include <cstdio>

#include <CMDCore/optparser>

#include <SfMFiles/sfmfiles>

int
main(int argc, const char* argv[])
{
    using namespace BDATA;
    using namespace cmdc;


    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("<in:model.patch> <in:model.out> <out:model.out>");
    optParser.addDescription("Converts from PMVS .patch file format to the bundler file format.");
    optParser.addFlag("dontLoadOption", "-p", "--dont-load-options", "Do not try to load options file for the PMVS file (in case PMVS cameras"
                      " indexes do not map to Bundler cameras)");
    optParser.addFlag("includeBadCameras", "-b", "--include-bad-cameras", "Include in the output bundle file the PMVS cameras that were labeled as bad.");
    optParser.parse(argc, argv);

    std::string pmvsFName = args[0];
    std::string inBundleFName = args[1];
    std::string outBundleFName = args[2];

    bool tryLoadOptions = opts["dontLoadOption"].asBool();
    bool includeBadCameras = opts["includeBadCameras"].asBool();

    LOG_EXPR(outBundleFName);
    LOG_EXPR(inBundleFName);
    LOG_EXPR(pmvsFName);

    LOG_INFO("Loading the bundle file");
    BDATA::BundlerData::Ptr bundle = BDATA::BundlerData::New(inBundleFName.c_str());

    LOG_INFO("Loading the pmvs file");
    PMVS::PMVSData::Ptr pmvs = PMVS::PMVSData::New(pmvsFName.c_str(), tryLoadOptions);

    LOG_INFO("Adding patches to bundle file");
    PMVS::Patch::Vector::iterator patch = pmvs->getPatches().begin();

    bundle->getPointInfo().resize(0);

    for(int i = 0; i < pmvs->getNPatches(); i++, patch++) {
        BDATA::PointInfo pinfo;

        // Position
        for(int j = 0; j < 3; j++) pinfo.position(j) = patch->position(j);

        // Color
        pinfo.color.r = (unsigned char) patch->color(0) * 255;
        pinfo.color.g = (unsigned char) patch->color(1) * 255;
        pinfo.color.b = (unsigned char) patch->color(2) * 255;

        // Cameras
        int nCams = patch->goodCameras.size();
        if(includeBadCameras) nCams += patch->badCameras.size();
        pinfo.viewList.resize(nCams);

        int camIdx = 0;
        for(int j = 0; j < patch->goodCameras.size(); j++, camIdx++) {
            pinfo.viewList[camIdx] = BDATA::PointEntry(patch->goodCameras[j], -1, Eigen::Vector2d());
        }

        if(includeBadCameras) {
            for(int j = 0; j < patch->badCameras.size(); j++, camIdx++) {
                pinfo.viewList[camIdx] = BDATA::PointEntry(patch->badCameras[j], -1, Eigen::Vector2d());
            }
        }

        bundle->getPointInfo().push_back(pinfo);
    }

    LOG_INFO("Writing " << outBundleFName);
    bundle->writeFile(outBundleFName.c_str());



    return EXIT_SUCCESS;
}

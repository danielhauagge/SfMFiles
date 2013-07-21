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

#define PRINT_MSG(msg) std::cout << ">> " << msg << std::endl

int
mainCameraMode(const BDATA::BundlerData bundle,
               const std::vector<std::string>& args,
               const std::map<std::string, std::string>& opts)
{
    using namespace BDATA;

    std::string fieldName = args[2];

    if (strcasecmp(fieldName.c_str(), "center") == 0) {
        Camera::Vector cams = bundle.getCameras();
        Camera::Vector::iterator cam = cams.begin();
        Eigen::Vector3d origCam = Eigen::Vector3d::Zero();
        for (; cam != cams.end(); cam++) {
            Eigen::Vector3d camCenter;
            cam->cam2world(origCam, camCenter);
            std::cout <<        std::setw(10) << camCenter[0]
                      << " " << std::setw(10) << camCenter[1]
                      << " " << std::setw(10) << camCenter[2] << std::endl;
        }
    }

    return EXIT_SUCCESS;
}

int
mainPointMode(const BDATA::BundlerData bundle,
              const std::vector<std::string>& args,
              const std::map<std::string, std::string>& opts)
{
    return EXIT_FAILURE;
}

int
main(int argc, const char** argv)
{
    using namespace BDATA;

    std::vector<std::string> args;
    std::map<std::string, std::string> opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("[OPTIONS] <in:bundle.out> CAM <field>");
    optParser.addUsage("[OPTIONS] <in:bundle.out> PNT <field>");
    optParser.addDescription("Prints miscelaneous information about a bundle file.");
    optParser.parse(argc, argv);

    std::string bundleFName = args[0];
    std::string mode = args[1];

    BundlerData bundle(bundleFName.c_str());

    if (strcasecmp(mode.c_str(), "cam") == 0) return mainCameraMode(bundle, args, opts);
    else if (strcasecmp(mode.c_str(), "pnt") == 0) return mainPointMode(bundle, args, opts);
    else {
        PRINT_MSG("ERROR: Incorrect usage, run with -h for help");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
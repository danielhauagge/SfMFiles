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
using namespace cmdc;

// Standard library
#include <iostream>
#include <iomanip>
#include <set>
#include <string>

// Boost
#include <boost/filesystem.hpp>

int
mainCameraMode(const Bundler::Reconstruction bundle,
               const OptionParser::Arguments &args,
               const OptionParser::Options &opts)
{
    using namespace Bundler;

    int w = 15;

    std::set<std::string> selFields;
    for(int i = 2; i < args.size(); i++) {
        selFields.insert(args[i]);
    }
    if(selFields.size() == 0) selFields.insert("all");

    Camera::Vector cams = bundle.getCameras();

    Camera::Vector::iterator cam = cams.begin();
    Camera::Vector::iterator camEnd = cams.end();
    int camIdx = 0;

    if(opts.count("selIdx")) {
        camIdx  = opts.at("selIdx").asInt();
        cam = cams.begin() + camIdx;
        camEnd = cam + 1;
    }

    std::string sep = "";
    for (; cam != camEnd; cam++, camIdx++) {
        std::cout << sep << "Camera " << camIdx << std::endl;
        sep = "\n";

        // Focal length
        if ( (selFields.count("im") || selFields.count("all") ) && bundle.listFileLoaded() ) {
            std::cout << std::setw(w) << "Image: " << bundle.getImageFileNames()[camIdx] << std::endl;
        }

        // Camera center in world coordinates
        if ( selFields.count("center") || selFields.count("all") ) {
            Eigen::Vector3d origCam = Eigen::Vector3d::Zero();
            Eigen::Vector3d camCenter;
            cam->cam2world(origCam, camCenter);

            std::cout << std::setw(w) << "Center: "
                      << "(" << camCenter[0]
                      << ", " << camCenter[1]
                      << ", " << camCenter[2] << ")" << std::endl;
        }

        // Camera aiming direction in world coordinates
        if ( selFields.count("lookat") || selFields.count("all") ) {
            Eigen::Vector3d lookAt;
            cam->lookingAt(lookAt);

            std::cout << std::setw(w) << "LookAt: "
                      << "(" << lookAt[0]
                      << ", " << lookAt[1]
                      << ", " << lookAt[2] << ")" << std::endl;
        }

        // Camera radial distortion parameters
        if ( selFields.count("rd") || selFields.count("all") ) {
            std::cout << std::setw(w) << "Radial coeffs: " << "k1 = " << cam->k1 << ", k2 = " << cam->k2 << std::endl;
        }

        // Focal length
        if ( selFields.count("fl") || selFields.count("all") ) {
            std::cout << std::setw(w) << "Focal length: " << cam->focalLength << std::endl;
        }

    }

    return EXIT_SUCCESS;
}

int
mainPointMode(const Bundler::Reconstruction bundle,
              const OptionParser::Arguments &args,
              const OptionParser::Options &opts)
{
    std::set<std::string> selFields;
    for(int i = 2; i < args.size(); i++) {
        selFields.insert(args[i]);
    }
    if(selFields.size() == 0) selFields.insert("all");

    Bundler::Point::Vector points = bundle.getPoints();

    Bundler::Point::Vector::iterator pnt = points.begin();
    Bundler::Point::Vector::iterator pntEnd = points.end();
    int pntIdx = 0;

    if(opts.count("selIdx")) {
        pntIdx  = opts.at("selIdx").asInt();
        pnt = points.begin() + pntIdx;
        pntEnd = pnt + 1;
    }

    std::string sep = "";
    for (; pnt != pntEnd; pnt++, pntIdx++) {
        std::cout << sep << "Point " << pntIdx << std::endl;
        sep = "\n";

        if ( selFields.count("pos") || selFields.count("all") ) {
            std::cout << " Position: " << pnt->position.transpose() << "\n";
        }

        if ( selFields.count("color") || selFields.count("all") ) {
            std::cout << "    Color: ["
                      << int(pnt->color.r) << ","
                      << int(pnt->color.g) << ","
                      << int(pnt->color.b) << "]\n";
        }

        if ( selFields.count("viewlist") || selFields.count("all") ) {
            Bundler::ViewListEntry::Vector::iterator vl = pnt->viewList.begin();
            Bundler::ViewListEntry::Vector::iterator vlEnd = pnt->viewList.end();

            std::cout << "View List:\n";
            std::string sepCam = "";
            for(; vl != vlEnd; vl++) {
                std::cout << sepCam;
                std::cout << "\t camera: " << vl->camera      << "\n";
                std::cout << "\t    key: " << vl->key         << "\n";
                std::cout << "\tkey pos: " << vl->keyPosition.transpose() << "\n";
                sepCam = "\n";
            }
        }
    }

    return EXIT_FAILURE;
}

int
main(int argc, const char **argv)
{
    using namespace Bundler;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.setNArguments(2, -1);
    optParser.addDescription("Prints miscelaneous information about a bundle file.");
    optParser.addUsage("<in:bundle.out> CAM <field>");
    optParser.addUsage("<in:bundle.out> PNT <field>");

    optParser.addOption("listFName", "-l", "F", "--list", "Bundler list filename");
    optParser.addOption("selIdx", "-i", "IDX", "--sel-idx", "Only print information from selected camera or point");

    optParser.parse(argc, argv);

    std::string bundleFName = args[0];
    std::string mode = args[1];

    // Load bundle file
    Reconstruction bundle(bundleFName.c_str());
    if(opts.count("listFName")) {
        bundle.readListFile(opts["listFName"].c_str());
    }

    // Print requested info
    if (strcasecmp(mode.c_str(), "cam") == 0) return mainCameraMode(bundle, args, opts);
    else if (strcasecmp(mode.c_str(), "pnt") == 0) return mainPointMode(bundle, args, opts);
    else {
        LOG_ERROR("Incorrect usage, run with -h for help");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
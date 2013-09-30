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

#include <SfMFiles/sfmfiles>
#include "ply.hpp"
#include "utils.hpp"
using namespace sfmf;
using namespace sfmf::Bundler;
#include <CMDCore/optparser>

#include <iostream>
#include <cstdio>
#include <cmath>

void
recolorByNCams(Reconstruction &bundler, std::string &colorMapping)
{
    std::vector<double> values(bundler.getNPoints());
    std::vector<double>::iterator v = values.begin();
    for(Point::Vector::const_iterator it = bundler.getPoints().begin(), itEnd = bundler.getPoints().end(); it != itEnd; it++, v++) {
        (*v) = it->viewList.size();
    }

    std::vector<Eigen::Vector3f> colors(bundler.getNPoints());
    colormapValues(values, colors, &colorMapping);

    std::vector<Eigen::Vector3f>::iterator c = colors.begin();
    for(Point::Vector::iterator it = bundler.getPoints().begin(), itEnd = bundler.getPoints().end(); it != itEnd; it++, v++, c++) {
        (*c) *= 255.0;
        it->color = Color((*c)[0], (*c)[1], (*c)[2]);
    }
}

void
colorPointsSeenByCamera(Reconstruction &bundler, int camIdx, std::string &colorMapping)
{
    Color red(250, 100, 100);
    Color green(100, 250, 100);
    for(Point::Vector::iterator it = bundler.getPoints().begin(), itEnd = bundler.getPoints().end(); it != itEnd; it++) {
        ViewListEntry::Vector::iterator pe = it->viewList.begin();
        ViewListEntry::Vector::iterator peEnd = it->viewList.end();
        bool found = false;
        for(; pe != peEnd && !found; pe++) if(pe->camera == camIdx) found = true;

        if(found) it->color = green;
        else it->color = red;
    }
}

int
main(int argc, const char *argv[])
{
    cmdc::Logger::setLogLevels(cmdc::LOGLEVEL_DEBUG);

    using namespace cmdc;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("<in:bundle.out> <out:bundle.ply>");
    optParser.addDescription("Convert Bundler reconstructions to the PLY file format.");
    // optParser.addFlag("colorByScore", "-s", "--color-score", "Change patches color to show the quality score.");
    optParser.addFlag("colorByNCams", "-c", "--color-n-cams", "Change point color to show the number of cameras a point sees.");
    optParser.addOption("camIdx", "-s", "IDX", "--points-seen-by-cam", "Change point color to show which points are seen by a given camera.", "-1");
    optParser.setNArguments(2, 2);
    optParser.parse(argc, argv);

    std::string bundleFName = args[0];
    std::string plyFName = args[1];

    // bool colorByScore = opts["colorByScore"].asBool();
    bool colorByNCams = opts["colorByNCams"].asBool();
    int camIdx = opts["camIdx"].asInt();

    Reconstruction bundler(bundleFName.c_str());

    Ply ply;
    std::stringstream comments;
    comments << "Input filename: " << bundleFName << "\n";

    if(colorByNCams) {
        comments << "Colored points based on number of visible cameras\n";
        std::string colorMapping;
        recolorByNCams(bundler, colorMapping);
        comments << colorMapping;
    } else if(camIdx >= 0) {
        comments << "Colored points that are visible to camera " << camIdx << "\n";
        std::string colorMapping;
        colorPointsSeenByCamera(bundler, camIdx, colorMapping);
        comments << colorMapping;
    }

    ply.addComment(comments.str());

    const Point::Vector &pnts = bundler.getPoints();
    for(Point::Vector::const_iterator it = pnts.begin(), itEnd = pnts.end(); it != itEnd; it++) {
        ply.addVertex(it->position, Ply::Color(it->color.r, it->color.g, it->color.b));
    }
    ply.writeToFile(plyFName);

    return EXIT_SUCCESS;
}

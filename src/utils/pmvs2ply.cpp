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
#include "utils.hpp"
#include "ply.hpp"
using namespace sfmf;

#include <CMDCore/optparser>

#include <iostream>
#include <cstdio>
#include <cmath>

void
recolorPatches(PMVS::Recontruction &patches, const std::vector<double> &values, std::string &colorMapping)
{
    std::vector<Eigen::Vector3f> colors(patches.getNPatches());
    colormapValues(values, colors, &colorMapping);

    std::vector<Eigen::Vector3f>::iterator c = colors.begin();
    for(PMVS::Patch::Vector::iterator patch = patches.getPatches().begin(); patch != patches.getPatches().end(); patch++, c++) {
        patch->color = *c;
    }
}

void
recolorByScore(PMVS::Recontruction &patches, std::string &colorMapping)
{
    std::vector<double> scores;
    for(PMVS::Patch::Vector::iterator patch = patches.getPatches().begin(); patch != patches.getPatches().end(); patch++) {
        scores.push_back(patch->score);
    }

    recolorPatches(patches, scores, colorMapping);
}

void
recolorByNumberOfCameras(PMVS::Recontruction &patches, bool useGoodCams, std::string &colorMapping)
{
    std::vector<double> patchNCams(patches.getNPatches());
    {
        int i = 0;
        for(PMVS::Patch::Vector::iterator patch = patches.getPatches().begin(); patch != patches.getPatches().end(); patch++, i++) {

            double ncams;
            if(useGoodCams) ncams = (double)patch->goodCameras.size();
            else ncams = (double)patch->badCameras.size();

            patchNCams[i] = ncams;
        }
    }

    recolorPatches(patches, patchNCams, colorMapping);
}

int
main(int argc, const char *argv[])
{
    cmdc::Logger::setLogLevels(cmdc::LOGLEVEL_DEBUG);
    using namespace cmdc;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("<in:model.patch> <out:model.ply>");
    optParser.addDescription("Utility for converting from PMVS's .patch file format to a .ply");
    optParser.addFlag("dontLoadOption", "-p", "--dont-load-options", "Do not try to load options file for the reconstruction (used to remap camera indexes)");
    optParser.addFlag("colorByScore", "-s", "--color-score", "Change patches color to show the quality score.");
    optParser.addFlag("colorByNCams", "-c", "--color-n-cams", "Change patches color to show the number of cameras a point sees.");
    optParser.addFlag("useGoodCams", "-b", "--use-bad-cams", "Use number of bad cameras to color points", 0, 1);
    optParser.setNArguments(2, 2);
    optParser.parse(argc, argv);

    std::string pmvsFName = args[0];
    std::string plyFName = args[1];

    bool tryLoadOptions = !opts["dontLoadOption"].asBool();
    bool colorByScore = opts["colorByScore"].asBool();
    bool colorByNCams = opts["colorByNCams"].asBool();
    bool useGoodCams = opts["useGoodCams"].asBool();

    PMVS::Recontruction pmvs(pmvsFName.c_str(), tryLoadOptions);
    Ply ply;

    std::stringstream comments;
    comments << "Input filename: " << pmvsFName << "\n";

    if(colorByScore) {
        std::string colorMapping;
        recolorByScore(pmvs, colorMapping);

        comments << "Colored patches based on score\n" << colorMapping;
    } else if(colorByNCams) {
        std::string colorMapping;
        recolorByNumberOfCameras(pmvs, useGoodCams, colorMapping);
        comments << "Colored patches based on number of cameras that see a point\n" << colorMapping;
    }

    ply.addComment(comments.str());

    PMVS::Patch::Vector::iterator patch = pmvs.getPatches().begin();;
    for(int i = 0; i < pmvs.getNPatches(); i++, patch++) {
        Eigen::Vector3d p(patch->position[0], patch->position[1], patch->position[2]);
        Eigen::Vector3d n(patch->normal[0], patch->normal[1], patch->normal[2]);
        Ply::Color c(patch->color[0] * 255, patch->color[1] * 255, patch->color[2] * 255);
        ply.addVertex(p, n, c);
    }

    ply.writeToFile(plyFName);

    return EXIT_SUCCESS;
}

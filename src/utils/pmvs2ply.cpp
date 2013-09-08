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
using namespace BDATA;
#include <CMDCore/optparser>

#include <iostream>
#include <cstdio>
#include <cmath>

void
recolorPatches(BDATA::PMVS::PMVSData& patches, const std::vector<double>& values)
{
    std::vector<double> valuesSorted = values;
    std::sort(valuesSorted.begin(), valuesSorted.end());
    double quants[4];
    quants[0] = valuesSorted[0];
    quants[1] = valuesSorted[patches.getNPatches() * 0.33];
    quants[2] = valuesSorted[patches.getNPatches() * 0.66];
    quants[3] = valuesSorted[patches.getNPatches() - 1];

    for (int i = 0; i < 4; i++) {
        LOG_EXPR(quants[i]);
    }

    Eigen::Vector3f quantColors[4] = {
        Eigen::Vector3f(0, 0, 1),
        Eigen::Vector3f(0, 1, 0),
        Eigen::Vector3f(1, 1, 0),
        Eigen::Vector3f(1, 0, 0)
    };

    const char* fmt = "%6.2f -> Color = [%.1f, %.1f, %.1f]\n";
    for (int i = 0; i < 4; i++) {
        printf(fmt, quants[i], quantColors[i][0], quantColors[i][1], quantColors[i][2]);
    }

    int patchIdx = 0;
    for(PMVS::Patch::Vector::iterator patch = patches.getPatches().begin(); patch != patches.getPatches().end(); patch++, patchIdx++) {
        double value = values[patchIdx];

        int i = 0;
        for(; value >= quants[i]; ++i);
        i = std::min(i, 3);
        assert(i >= 0 && i < 4);

        float alpha = float(value - quants[i - 1]) / float(quants[i] - quants[i - 1]);

        assert(alpha <= 1.0 && alpha >= 0.0);

        patch->color = quantColors[i - 1] * alpha + (1.0 - alpha) * quantColors[i];
    }
}

void
recolorByScore(BDATA::PMVS::PMVSData& patches, bool stretchValues = false)
{
    using namespace BDATA;

    stretchValues = true;

    std::vector<double> scores;
    for(PMVS::Patch::Vector::iterator patch = patches.getPatches().begin(); patch != patches.getPatches().end(); patch++) {
        scores.push_back(patch->score);
    }

    recolorPatches(patches, scores);
}

void
recolorByNumberOfCameras(BDATA::PMVS::PMVSData& patches, bool useGoodCams)
{
    LOG_EXPR(useGoodCams);

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

    recolorPatches(patches, patchNCams);
}

int
main(int argc, const char* argv[])
{
    cmdc::Logger::setLogLevels(cmdc::LOGLEVEL_DEBUG);

    using namespace BDATA;
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

    PMVS::PMVSData pmvs(pmvsFName.c_str(), tryLoadOptions);

    if(colorByScore) recolorByScore(pmvs);
    if(colorByNCams) recolorByNumberOfCameras(pmvs, useGoodCams);

    // Write PLY header
    //FILE* plyF = fopen(plyFName.c_str(), "wb");
    FILE* plyF = fopen(plyFName.c_str(), "w");
    if(plyF == NULL) {
        std::cout << "ERROR: Could not open file " << plyFName << " for writing" << std::endl;
        return EXIT_FAILURE;
    }
    fprintf(plyF, "ply\n");
    //fprintf(plyF, "format binary_little_endian 1.0\n");
    fprintf(plyF, "format ascii 1.0\n");
    fprintf(plyF, "element vertex %d\n", int(pmvs.getNPatches()));
    fprintf(plyF, "property float x\nproperty float y\nproperty float z\n");
    fprintf(plyF, "property uchar diffuse_red\nproperty uchar diffuse_green\nproperty uchar diffuse_blue\nend_header\n");

    LOG_INFO("Processing data");
    PMVS::Patch::Vector::iterator patch = pmvs.getPatches().begin();;
    for(int i = 0; i < pmvs.getNPatches(); i++, patch++) {
        for(int j = 0; j < 3; j++) {
            float coord = patch->position[j];
            //fwrite(&coord, sizeof(float), 1, plyF);
            fprintf(plyF, "%f ", coord);
        }

        for(int j = 0; j < 3; j++) {
            double color = patch->color[j];
            //fwrite(&color, sizeof(float), 1, plyF);
            fprintf(plyF, "%d ", (int)round(color * 255.0));
        }
        fprintf(plyF, "\n");
    }

    fclose(plyF);



    return EXIT_SUCCESS;
}

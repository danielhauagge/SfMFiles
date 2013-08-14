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

#include <OptParser/optparser>

#include <iostream>
#include <cstdio>
#include <cmath>

#define LOG(msg) std::cout << ">> " << msg << std::endl;
//#define PRINT_EXPR(expr) LOG(#expr << " = " << (expr))

void
recolorByScore(BDATA::PMVS::PMVSData& patches, bool stretchValues = false)
{
    using namespace BDATA;

    stretchValues = true;

    double minScore = -1.0, maxScore = 1.0;
    if(stretchValues) {
        minScore = 2.0, maxScore = -2.0;
        for(PMVS::Patch::Vector::iterator patch = patches.getPatches().begin(); patch != patches.getPatches().end(); patch++) {
            minScore = std::min(minScore, patch->score);
            maxScore = std::max(maxScore, patch->score);
        }
    }

    for(PMVS::Patch::Vector::iterator patch = patches.getPatches().begin(); patch != patches.getPatches().end(); patch++) {
        float score = patch->score;
        score -= minScore;
        score /= (maxScore - minScore);
        //PRINT_EXPR(score);

        float r, g, b;
        r = score;
        g = 1.0 - score;
        b = 0.0;

        patch->color[0] = r;
        patch->color[1] = g;
        patch->color[2] = b;
    }
}

void
recolorByNumberOfCameras(BDATA::PMVS::PMVSData& patches)
{
    using namespace BDATA;

    size_t minNCams = 1000000, maxNCams = 0;
    for(PMVS::Patch::Vector::iterator patch = patches.getPatches().begin(); patch != patches.getPatches().end(); patch++) {
        minNCams = std::min(minNCams, patch->goodCameras.size());
        maxNCams = std::max(maxNCams, patch->goodCameras.size());
    }
    std::cout << "min # cams: " << minNCams << std::endl;
    std::cout << "max # cams: " << maxNCams << std::endl;

    for(PMVS::Patch::Vector::iterator patch = patches.getPatches().begin(); patch != patches.getPatches().end(); patch++) {
        int nCams = patch->goodCameras.size();
        float score = nCams;
        score -= minNCams;
        score /= (maxNCams - minNCams);
        //PRINT_EXPR(score);

        float r, g, b;
        if(nCams < 10) {
            r = 1;
            g = 0;
            b = 0;
        } else if(nCams < 20) {
            r = 0;
            g = 1;
            b = 0;
        } else {
            r = 0;
            g = 0;
            b = 1;
        }

        //float r, g, b;
        //r = score;
        //g = 1.0 - score;
        //b = 0.0;

        patch->color[0] = r;
        patch->color[1] = g;
        patch->color[2] = b;
    }
}

#if 0
int
round(double x)
{
    return int(x + 0.5);
}
#endif

int
main(int argc, const char* argv[])
{
    using namespace BDATA;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("<in:model.patch> <out:model.ply>");
    optParser.addDescription("Utility for converting from PMVS's .patch file format to a .ply");
    optParser.addFlag("dontLoadOption", "-p", "--dont-load-options", "Do not try to load options file for the reconstruction (used to remap camera indexes)");
    optParser.addFlag("colorByScore", "-s", "--color-score", "Change patches color to show the quality score.");
    optParser.addFlag("colorByNCams", "-c", "--color-n-cams", "Change patches color to show the number of cameras a point sees.");
    optParser.setNArguments(2, 2);
    optParser.parse(argc, argv);

    std::string pmvsFName = args[0];
    std::string plyFName = args[1];

    bool tryLoadOptions = !opts["dontLoadOption"].asBool();
    bool colorByScore = opts["colorByScore"].asBool();
    bool colorByNCams = opts["colorByNCams"].asBool();

    PMVS::PMVSData pmvs(pmvsFName.c_str(), tryLoadOptions);

    if(colorByScore) recolorByScore(pmvs);
    if(colorByNCams) recolorByNumberOfCameras(pmvs);

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

    PRINT_MSG("Processing data");
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

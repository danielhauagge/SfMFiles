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

void
recolorByScore(BDATA::PMVS::PMVSData& patches)
{
    using namespace BDATA;

    // Color scheme
    // from 0 to  1: white to green
    // from 0 to -1: white to red

    for(PMVS::Patch::Vector::iterator patch = patches.getPatches().begin(); patch != patches.getPatches().end(); patch++) {
        float score = patch->score;

        float r, g, b;
        if(score <= 0) {
            r = 1.0;
            g = 1.0 + score;
            b = 1.0 + score;
        } else {
            r = 1.0 - score;
            g = 1.0;
            b = 1.0 - score;
        }

        patch->color[0] = r;
        patch->color[1] = g;
        patch->color[2] = b;
    }
}

int
main(int argc, const char* argv[])
{
    using namespace BDATA;

    std::vector<std::string> args;
    std::map<std::string, std::string> opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("[OPTIONS] <in:model.patch> <out:model.ply>");
    optParser.addDescription("Utility for converting from PMVS's .patch file format to a .ply");
    optParser.addFlag("dontLoadOption", "-p", "--dont-load-options", "Do not try to load options file for the reconstruction (used to remap camera indexes)");
    optParser.addFlag("colorByScore", "-s", "--colorScore", "Change patches color to show the quality score.");
    optParser.parse(argc, argv);

    std::string pmvsFName = args[0];
    std::string plyFName = args[1];

    bool tryLoadOptions = !atoi(opts["dontLoadOption"].c_str());
    bool colorByScore = atoi(opts["colorByScore"].c_str());

    PMVS::PMVSData pmvs(pmvsFName.c_str(), tryLoadOptions);

    if(colorByScore) recolorByScore(pmvs);

    // Write PLY header
    FILE* plyF = fopen(plyFName.c_str(), "wb");
    if(plyF == NULL) {
        std::cout << "ERROR: Could not open file " << plyFName << " for writing" << std::endl;
        return EXIT_FAILURE;
    }
    fprintf(plyF, "ply\n");
    fprintf(plyF, "format binary_little_endian 1.0\n");
    fprintf(plyF, "element vertex %d\n", int(pmvs.getNPatches()));
    fprintf(plyF, "property float x\nproperty float y\nproperty float z\n");
    fprintf(plyF, "property float red\nproperty float green\nproperty float blue\nend_header\n");

    PRINT_MSG("Processing data");
    PMVS::Patch::Vector::iterator patch = pmvs.getPatches().begin();;
    for(int i = 0; i < pmvs.getNPatches(); i++, patch++) {
        for(int j = 0; j < 3; j++) {
            float coord = patch->position[j];
            fwrite(&coord, sizeof(float), 1, plyF);
            //fprintf(plyF, "%f ", coord);
        }

        for(int j = 0; j < 3; j++) {
            float color = patch->color[j];
            //LOG(color);
            fwrite(&color, sizeof(float), 1, plyF);
        }

        //fprintf(plyF, "\n");
    }

    fclose(plyF);

    return EXIT_SUCCESS;
}

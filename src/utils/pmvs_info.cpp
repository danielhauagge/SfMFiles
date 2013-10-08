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
using namespace sfmf;
#include <CMDCore/optparser>
using namespace cmdc;

// const char* pointFields[8] = {
//     "score",
//     "ngoodcams",
//     "nbadcams",
//     "coords",
//     "normal",
//     "color",
//     "accuracy",
//     "sevel"
// };

int
mainPointMode(const PMVS::Reconstruction pmvs,
              const OptionParser::Arguments &args,
              const OptionParser::Options &opts)
{
    // Which fields did the user specify
    std::set<std::string> selFields;
    for(int i = 2; i < args.size(); i++) selFields.insert(args[i]);
    if(selFields.size() == 0) selFields.insert("all");

    std::ostream *out = &std::cout;
    std::ofstream outF;
    if(opts.count("outFName")) {
        std::string outFName = opts.at("outFName");
        outF.open(outFName.c_str());;
        if(!outF.good()) {
            LOG_ERROR("Could not open file " << outFName << " for reading");
        }
        out = &outF;
    }

    PMVS::Patch::Vector patches = pmvs.getPatches();

    PMVS::Patch::Vector::iterator patch = patches.begin();
    PMVS::Patch::Vector::iterator patchEnd = patches.end();
    int pntIdx = 0;

    if(opts.count("selIdx")) {
        pntIdx  = opts.at("selIdx").asInt();
        patch = patches.begin() + pntIdx;
        patchEnd = patch + 1;
    }

    std::string sep = "";
    for (int i = 0; patch != patchEnd; patch++, pntIdx++, i++) {
        (*out) << pntIdx << " ";

        // Score
        if ( (selFields.count("score") || selFields.count("all") ) ) {
            (*out) << patch->score << " ";
        }

        // Number of good cameras
        if ( (selFields.count("ngoodcams") || selFields.count("all") ) ) {
            (*out) << patch->goodCameras.size() << " ";
        }

        // Number of bad cameras
        if ( (selFields.count("nbadcams") || selFields.count("all") ) ) {
            (*out) << patch->badCameras.size() << " ";
        }

        // Coordinates
        if ( (selFields.count("coords") || selFields.count("all") ) ) {
            (*out) << patch->position.transpose() << " ";
        }

        // Normal
        if ( (selFields.count("normal") || selFields.count("all") ) ) {
            (*out) << patch->normal.transpose() << " ";
        }

        // Normal
        if ( (selFields.count("color") || selFields.count("all") ) ) {
            (*out) << patch->color.transpose() << " ";
        }

        // Reconstruction accuracy
        if ( (selFields.count("accuracy") || selFields.count("all") ) ) {
            (*out) << patch->reconstructionAccuracy << " ";
        }

        // Reconstruction reconstructionSLevel
        if ( (selFields.count("slevel") || selFields.count("all") ) ) {
            (*out) << patch->reconstructionSLevel << " ";
        }

        (*out) << std::endl;
    }

    return EXIT_SUCCESS;
}


int
main(int argc, char const *argv[])
{
    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.setNArguments(2, -1);
    optParser.addDescription("Prints miscelaneous information about a PMVS .patch file.");
    optParser.addUsage("<in:reconstruction.patch> PNT <field>");
    //optParser.addUsage("<in:reconstruction.patch> CAM <field>");
    optParser.addOption("selIdx", "-i", "IDX", "--sel-idx", "Only print information from selected camera or point");
    optParser.addOption("outFName", "-o", "F", "", "Output information to file");

    optParser.parse(argc, argv);

    std::string patchFName = args[0];
    std::string mode = args[1];

    PMVS::Reconstruction pmvs(patchFName.c_str());
    pmvs.loadCamerasAndImageFilenames();

    // Print requested info
    //if (strcasecmp(mode.c_str(), "cam") == 0) return mainCameraMode(bundle, args, opts);
    if (strcasecmp(mode.c_str(), "pnt") == 0) return mainPointMode(pmvs, args, opts);
    else {
        LOG_ERROR("Incorrect usage, run with -h for help");
        return EXIT_FAILURE;
    }


    return EXIT_SUCCESS;
}

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

#include <fstream>

int
main(int argc, char const *argv[])
{
    cmdc::Logger::setLogLevels(cmdc::LOGLEVEL_DEBUG);

    using namespace Bundler;
    using namespace cmdc;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("<in:model.patch> <in:transform.txt> <out:model_transformed.patch>");
    optParser.addDescription("Apply transform to points in a PMVS .patch file, cameras are not changed. Transform is "
                             "supplied as a 3 x 4 matrix stored in a text file.");
    optParser.setNArguments(3, 3);
    optParser.parse(argc, argv);

    std::string inPMVSFname    = args[0];
    std::string transformFName = args[1];
    std::string outPMVSFName   = args[2];

    LOG_INFO("Loading transform");
    std::ifstream fTrans(transformFName.c_str());
    if(fTrans.bad()) {
        LOG_ERROR("Could not open file " << transformFName << " for reading");
        return EXIT_FAILURE;
    }
    Eigen::MatrixXd trans(4, 4);
    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            float v;
            fTrans >> v;
            trans(i, j) = v;
        }
    }
    for (int j = 0; j < 3; j++) trans(3, j) = 0.0;
    trans(3, 3) = 1.0;
    Eigen::MatrixXd rot = trans.block(0, 0, 3, 3);

    if(fabs(rot.determinant() - 1.0) > 0.0001) {
        LOG_ERROR("Bad rotation matrix (determinant is not 1)");
        return EXIT_FAILURE;
    }

    LOG_INFO("Loading PMVS file");
    PMVS::Reconstruction pmvs(inPMVSFname.c_str());

    PROGBAR_START("Applying transform to all points");
    PMVS::Patch::Vector &patches = pmvs.getPatches();
    PMVS::Patch::Vector::iterator patch = patches.begin();
    for (int i = 0, iEnd = pmvs.getNPatches(); i < iEnd; i++, patch++) {
        PROGBAR_UPDATE(i, iEnd);

        patch->position = trans * patch->position;
        patch->normal   = trans * patch->normal;
    }

    LOG_INFO("Writing output to " << outPMVSFName);
    pmvs.writeFile(outPMVSFName.c_str());

    return EXIT_SUCCESS;
}
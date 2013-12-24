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

#include <algorithm>    // std::set_union, std::sort

int
main(int argc, char const *argv[])
{
    Logger::setLogLevels(LOGLEVEL_NOPRINTING);

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.setNArguments(1, -1);
    optParser.addDescription("Checks to see if .patch file camera indexes have already been remapped with options file. Returns 1 if file seems to have been remapped, 0 othewise.");
    optParser.addUsage("<in:model.patch>");
    optParser.addFlag("verbose", "-v", "--verbose", "Print various messages");

    optParser.parse(argc, argv);

    std::string patchFName = args[0];
    if(opts["verbose"].asBool()) {
        Logger::setLogLevels(LOGLEVEL_INFO);
    }

    // load reconstruction
    PMVS::Reconstruction pmvs(patchFName.c_str(), false);

    // load options file
    LOG_INFO("Loading options file");
    PMVS::Options pmvsOpts;
    std::string pmvsOptsFName = PMVS::Reconstruction::defaultOptionsFileForPatchFile(patchFName);
    std::ifstream pmvsOptsF(pmvsOptsFName.c_str());
    pmvsOptsF >> pmvsOpts;

    // gather all camera indexes in reconstruction
    LOG_INFO("gather all camera indexes in reconstruction");
    std::set<uint32_t> goodCamIdxs;
    for(PMVS::Patch::Vector::const_iterator patch = pmvs.getPatches().begin(); patch != pmvs.getPatches().end(); patch++) {
        for(std::vector<uint32_t>::const_iterator cam = patch->goodCameras.begin(); cam != patch->goodCameras.end(); cam++) {
            goodCamIdxs.insert(*cam);
        }
    }

    LOG_INFO("Computing intersection");
    std::set<uint32_t> idxsUnion;//(goodCamIdxs.size() + pmvsOpts.timages.size());
    goodCamIdxs.insert(pmvsOpts.timages.begin(), pmvsOpts.timages.end());

    LOG_INFO("idxsUnion.size() = " << idxsUnion.size());
    LOG_INFO("goodCamIdxs.size() = " << goodCamIdxs.size());
    LOG_INFO("pmvsOpts.timages.size() = " << pmvsOpts.timages.size());

    if(goodCamIdxs.size() != pmvsOpts.timages.size()) {
        std::cout << "0" << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cout << "1" << std::endl;
        return EXIT_SUCCESS;
    }
}
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

// Other projects
#include <SfMFiles/sfmfiles>
using namespace sfmf;
#include <CMDCore/optparser>

// STD
#include <iostream>
#include <iomanip>

int
main(int argc, const char **argv)
{
    using namespace PMVS;
    using namespace cmdc;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("<out:model.patch> <in:model1.patch> <in:model2.patch>...");
    optParser.addDescription("Merge multiple PMVS patch files into a larger one.");
    optParser.addFlag("dontLoadOption", "-p", "--dont-load-options", "Do not try to load options file for the reconstruction (used to remap camera indexes)");
    optParser.parse(argc, argv);

    std::string outFName = args[0];
    bool tryLoadOptions = !opts["dontLoadOption"].asBool();

    std::vector<std::string> inFNames;
    for (int i = 1; i < args.size(); i++) {
        inFNames.push_back(args[i]);
    }

    Recontruction merged;
    for (int i = 0; i < inFNames.size(); i++) {
        std::cout << "[" << std::setw(4) << i << "/" << inFNames.size() << "] " << inFNames[i] << std::endl;
        Recontruction pmvs(inFNames[i].c_str(), tryLoadOptions);
        merged.mergeWith(pmvs);
    }

    LOG_INFO("Writing file " << outFName);
    merged.writeFile(outFName.c_str());


    return EXIT_SUCCESS;
}

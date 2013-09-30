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

#include <SfMFiles/sfmfiles>
#include <CMDCore/optparser>

int
main(int argc, char const *argv[])
{
    cmdc::Logger::setLogLevels(cmdc::LOGLEVEL_DEBUG);

    using namespace sfmf;
    using namespace Bundler;
    using namespace cmdc;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("<in:bundle.out> <in:list.txt> <out:new_list.txt>");
    optParser.addDescription("Generates a new list file with focal length for all valid cameras.");
    optParser.parse(argc, argv);

    std::string bundleFName = args[0];
    std::string inListFName = args[1];
    std::string outListFName = args[2];

    Bundler::Reconstruction bundler(bundleFName.c_str());
    bundler.readListFile(inListFName.c_str());

    std::ofstream outList(outListFName.c_str());
    if(outList.bad()) {
        LOG_ERROR("Could not open file " << outListFName << " for writing");
        return EXIT_FAILURE;
    }

    Bundler::Camera::Vector::iterator cam = bundler.getCameras().begin();
    for (int camIdx = 0; camIdx < bundler.getNCameras(); camIdx++, cam++) {
        int imW, imH;
        double focal = 0.0;
        if(cam->isValid()) focal = cam->focalLength;

        outList << bundler.getImageFileNames()[camIdx] << " 0 " << focal << "\n";
    }

    return EXIT_SUCCESS;
}
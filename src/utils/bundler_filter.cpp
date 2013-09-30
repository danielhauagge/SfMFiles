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
using namespace sfmf;

#include <CMDCore/optparser>

void
filterByNCams(Bundler::Reconstruction &bundler, int minNCams)
{
    using namespace Bundler;

    Point::Vector newPnts;
    PROGBAR_START("Processing points");
    int idx = 0;
    int nPnts = bundler.getNPoints();
    int nCulled = 0;
    for(Point::Vector::iterator it = bundler.getPoints().begin(), itEnd = bundler.getPoints().end(); it != itEnd; it++, idx++) {
        PROGBAR_UPDATE(idx, nPnts);
        if(it->viewList.size() >= minNCams) {
            newPnts.push_back(*it);
        } else {
            nCulled++;
        }
    }

    LOG_INFO(nCulled << "/" << bundler.getNPoints() << " points were removed");

    bundler.getPoints() = newPnts;
}

int
main(int argc, char const *argv[])
{
    using namespace Bundler;
    using namespace cmdc;

    OptionParser::Arguments args;
    OptionParser::Options opts;

    OptionParser optParser(&args, &opts);
    optParser.addUsage("<in:old.out> <out:new.out>");
    optParser.addDescription("Utility that provides multiple ways of filtering points from a bundler file.");
    // optParser.addOption("boundingSphere", "", "S", "--bounding-sphere",
    //                     "Keep all points that fall within a bounding sphere. Specify as X,Y,Z,R where "
    //                     "X, Y, and Z are center coordinates and R is the radius.");
    optParser.addOption("minNCams", "-n", "N", "--by-ncams",
                        "Keel all points that are seen by at least N cameras", "-1");
    optParser.parse(argc, argv);

    std::string inBundleFName = args[0];
    std::string outBundleFName = args[1];

    int minNCams = opts["minNCams"].asInt();

    Reconstruction bundler(inBundleFName.c_str());

    filterByNCams(bundler, minNCams);

    LOG_INFO("Writing output to " << outBundleFName);
    bundler.writeFile(outBundleFName.c_str());

    return EXIT_SUCCESS;
}
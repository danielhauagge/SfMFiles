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
    optParser.addUsage("<out:bundle.out> <in:transform.txt> <out:bundle2.out>");
    optParser.addDescription("Apply transform to a bundler file. Transform is supplied as a 3 x 4 matrix stored in a text file.");
    optParser.setNArguments(3, 3);
    optParser.parse(argc, argv);

    std::string inBundleFName = args[0];
    std::string transformFName = args[1];
    std::string outBundleFName = args[2];

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

    LOG_INFO("Loading bundle file");
    Bundler::Reconstruction bundle(inBundleFName.c_str());

    PROGBAR_START("Applying transform to all points");
    Bundler::Point::Vector &pnts = bundle.getPoints();
    Bundler::Point::Vector::iterator pnt = pnts.begin();
    for (int i = 0, iEnd = bundle.getNPoints(); i < iEnd; i++, pnt++) {
        PROGBAR_UPDATE(i, iEnd);

        Eigen::Vector4d p(pnt->position[0], pnt->position[1], pnt->position[2], 1.0);
        p = trans * p;

        for (int j = 0; j < 3; j++) {
            pnt->position[j] = p[j];
        }
    }

    Eigen::MatrixXd transInv = trans.inverse();
    PROGBAR_START("Applying transform to all cameras");
    Bundler::Camera::Vector &cams = bundle.getCameras();
    Bundler::Camera::Vector::iterator cam = cams.begin();
    for (int i = 0, iEnd = bundle.getNCameras(); i < iEnd; i++, cam++) {
        PROGBAR_UPDATE(i, iEnd);
        Eigen::MatrixXd camT(4, 4);
        camT.block(0, 0, 3, 3) = cam->rotation;
        camT.block(0, 3, 3, 1) = cam->translation;
        camT.block(3, 0, 1, 3).setZero();
        camT(3, 3) = 1;

        camT *= transInv;

        cam->rotation = camT.block(0, 0, 3, 3);
        cam->translation = camT.block(0, 3, 3, 1);
    }

    LOG_INFO("Writing output to " << outBundleFName);
    bundle.writeFile(outBundleFName.c_str());

    return EXIT_SUCCESS;
}
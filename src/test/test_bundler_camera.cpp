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

const char* cam1Str = "500.00000000000000000000 0.00000000000000000000 0.00000000000000000000\n"
                      "1.00000000000000000000 -0.00000000000000000000 0.00000000000000000000 \n"
                      "-0.00000000000000000000 -0.00000016292068494295 1.00000000000000000000 \n"
                      "0.00000000000000000000 -1.00000000000000000000 -0.00000016292068494295 \n"
                      "-0.00000000000000000000 -0.00000065168273977179 -4.00000000000000000000 \n";

int
test1(int argc, char const* argv[])
{
    std::cout << "Taka a point that is in front of the camera to image coordinates and back and make sure that both lie on the same side of the camera" << std::endl;
    std::istringstream cam1S(cam1Str);
    BDATA::Camera cam;
    int width = 500, height = 500;
    cam1S >> cam;

    Eigen::Vector3d pntW(0, 0, 0);
    std::cout << "Point World Ori: " << pntW[0] << ", " << pntW[1]  << ", " << pntW[2] << std::endl;

    Eigen::Vector2d pntIm;
    cam.world2im(pntW, pntIm, false, width, height);
    std::cout << "    Point Image: " << pntIm[0] << ", " << pntIm[1] << std::endl;
    assert(fabs(pntIm[0] - 250) < 0.01);
    assert(fabs(pntIm[1] - 250) < 0.01);

    Eigen::Vector3d pntW2;
    cam.im2world(pntIm, pntW2, width, height);
    std::cout << "Point World Rec: " << pntW2[0] << ", " << pntW2[1]  << ", " << pntW2[2] << std::endl;

    Eigen::Vector3d camCenter;
    cam.center(camCenter);
    double dotProd = (pntW - camCenter).dot(pntW2 - camCenter);
    std::cout << "       Dot prod: " << dotProd << std::endl;
    assert(dotProd > 0); // Reconstructed point should like on the same side of the cam

    return EXIT_SUCCESS;
}

int
test2(int argc, char const* argv[])
{
    LOG_INFO("Camera up vector");
    BDATA::Camera cam;
    Eigen::Vector3d camUp;
    cam.up(camUp);

    assert((camUp - Eigen::Vector3d(0, 1, 0)).norm() < 0.001);

    Eigen::Vector2d imCenterIm(0, 0), imTopIm(0, 1);
    Eigen::Vector3d imCenterW, imTopW;
    cam.im2world(imCenterIm, imCenterW);
    cam.im2world(imTopIm, imTopW);

    Eigen::Vector3d camUpFromIm = imTopW - imCenterW;
    camUpFromIm /= camUpFromIm.norm();
    assert((camUpFromIm - Eigen::Vector3d(0, 1, 0)).norm() < 0.01);

    return EXIT_SUCCESS;
}

int
test3(int argc, char const* argv[])
{
    LOG_INFO("Camera lookAt vector");

    BDATA::Camera cam;
    Eigen::Vector3d lookAt;
    cam.lookingAt(lookAt);
    assert((lookAt - Eigen::Vector3d(0, 0, -1)).norm() < 0.01);

    return EXIT_SUCCESS;
}

int
main(int argc, char const* argv[])
{
    cmdc::init();
    cmdc::Logger::setLogLevels(cmdc::LOGLEVEL_DEBUG);

    if(argc == 1) {
        std::cout << "Usage:\n\t" << argv[0] << " <in:testnum>" << std::endl;
        return EXIT_FAILURE;
    }

    int testNum = atoi(argv[1]);

    switch(testNum) {
    case 1:
        return test1(argc - 2, &argv[2]);
        break;
    case 2:
        return test2(argc - 2, &argv[2]);
        break;
    case 3:
        return test3(argc - 2, &argv[2]);
        break;
    default:
        LOG_WARN("Invalid number for test " << testNum);
    }
    cmdc::deinit();

    return EXIT_SUCCESS;
}
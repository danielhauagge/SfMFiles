#include <SfMFiles/sfmfiles>

int
test1(int argc, char const* argv[])
{
    const char* cam1Str = "500.00000000000000000000 0.00000000000000000000 0.00000000000000000000\n"
                          "1.00000000000000000000 -0.00000000000000000000 0.00000000000000000000 \n"
                          "-0.00000000000000000000 -0.00000016292068494295 1.00000000000000000000 \n"
                          "0.00000000000000000000 -1.00000000000000000000 -0.00000016292068494295 \n"
                          "-0.00000000000000000000 -0.00000065168273977179 -4.00000000000000000000 \n";

    LOG_INFO("Taka a point that is in front of the camera to image coordinates and back and make sure that both lie on the same side of the camera");
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
test4(int argc, char const* argv[])
{
    LOG_INFO("Testing world2im");
    const char* camStr = "7.0008849479e+02 -7.0992716605e-02 -2.8653295186e-02\n"
                         "9.9240045398e-01 -1.1447615454e-01 4.5128139672e-02\n"
                         "9.6516563784e-02 9.5165945078e-01 2.9159705528e-01\n"
                         "-7.6327530178e-02 -2.8502543707e-01 9.5547611606e-01\n"
                         "1.8342005790e-01 9.7561838757e-01 -8.2822559093e-01";

    std::istringstream camS(camStr);
    BDATA::Camera cam;
    camS >> cam;

    Eigen::Vector3d pntW(-0.134889, -0.827748, -2.69439);
    Eigen::Vector2d pntIm(5.0258, -135.11);
    Eigen::Vector2d pntImComputed;

    cam.world2im(pntW, pntImComputed, true);

    double err = (pntImComputed - pntIm).norm();
    assert(err < 0.0005);

    return EXIT_SUCCESS;
}

int
main(int argc, char const* argv[])
{
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
    case 4:
        return test4(argc - 2, &argv[2]);
        break;
    default:
        LOG_WARN("Invalid number for test " << testNum);
    }


    return EXIT_SUCCESS;
}
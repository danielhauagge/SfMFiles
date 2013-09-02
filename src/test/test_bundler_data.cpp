#include <SfMFiles/sfmfiles>

int
main(int argc, const char* argv[])
{
    using namespace BDATA;
    cmdc::init();

    if(argc == 1) {
        std::cout << "Usage:\n\t" << argv[0] << " <bundle.out> <list.txt> <cam index>" << std::endl;
        return EXIT_FAILURE;
    }

    const char* bundleFName = argv[1];
    const char* listFName = argv[2];
    int camNum = atoi(argv[3]);

    LOG_INFO("Loading bundle file");
    BDATA::BundlerData bundler(bundleFName);
    LOG_EXPR(bundler.getNCameras());
    LOG_EXPR(bundler.getNValidCameras());
    LOG_EXPR(bundler.getNPoints());

    LOG_INFO("Building camera to point index");
    bundler.buildCam2PointIndex();

    try {
        LOG_INFO("Loading list file");
        bundler.readListFile(listFName);
    } catch (sfmf::Error e) {
        LOG_WARN("Caught exception");
        LOG_WARN(" WHAT: " << e.what());
    }

    LOG_EXPR(bundler.getListFileName());
    LOG_EXPR(bundler.getImageFileNames()[camNum]);

    // Test transforms
    PointInfo& pntInfo = bundler.getPointInfo()[0];

    LOG_EXPR(pntInfo.position.transpose());
    const int pntIdx = 0;
    Eigen::Vector2d featPos = pntInfo.viewList[pntIdx].keyPosition;

    printf("\n>> TESTING COORDINATE TRANSFORMS ====================\n");
    printf(">> World -> Image\n");

    Eigen::Vector2d predFeatPos;
    bundler.getCameras()[pntInfo.viewList[pntIdx].camera].world2im(pntInfo.position, predFeatPos, true);

    printf(">>\t     featPos = [%3.2f, %3.2f]\n", featPos(0), featPos(1));
    printf(">>\t predFeatPos = [%3.2f, %3.2f]\n", predFeatPos(0), predFeatPos(1));
    printf(">>\t         err = %f\n", (predFeatPos - featPos).norm());

    printf("\n>> Image -> Cam\n");
    Eigen::Vector3d featPosC;
    bundler.getCameras()[pntInfo.viewList[pntIdx].camera].im2cam(featPos, featPosC);
    Eigen::Vector3d projPosition;
    bundler.getCameras()[pntInfo.viewList[pntIdx].camera].world2cam(pntInfo.position, projPosition);
    projPosition /= projPosition(2);

    printf(">>\t    featPosC = [%3.4f, %3.4f, %3.4f]\n", featPosC(0), featPosC(1), featPosC(2));
    printf(">>\tprojPosition = [%3.4f, %3.4f, %3.4f]\n", projPosition(0), projPosition(1), projPosition(2));
    printf(">>\t         err = %f\n", (featPosC - projPosition).norm());

    printf("\n>> Image -> Cam -> Image\n");
    Eigen::Vector2d featPosCI;
    bundler.getCameras()[pntInfo.viewList[pntIdx].camera].cam2im(featPosC, featPosCI, false);

    printf(">>\t     featPos = [%3.2f, %3.2f]\n", featPos(0), featPos(1));
    printf(">>\t   featPosCI = [%3.2f, %3.2f]\n", featPosCI(0), featPosCI(1));
    printf(">>\t         err = %f\n", (featPosCI - featPos).norm());

    cmdc::deinit();
    return EXIT_SUCCESS;
}

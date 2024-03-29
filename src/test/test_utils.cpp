#undef NDEBUG

#include <SfMFiles/sfmfiles>
using namespace sfmf;

#include <utils.hpp>

int
test1(int argc, char const *argv[])
{
    LOG_INFO("Test get image size");

    const char *imFName = argv[0];

    int width, height;
    if(getImageSize(imFName, width, height)) {
        LOG_EXPR(width);
        LOG_EXPR(height);
    } else {
        LOG_WARN("Could not determine size of image " << imFName);
    }

    return EXIT_SUCCESS;
}

int
main(int argc, char const *argv[])
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
    default:
        LOG_WARN("No test " << testNum);
    }

    return EXIT_SUCCESS;
}
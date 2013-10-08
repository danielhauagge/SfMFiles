#undef NDEBUG

#include "SfMFiles/sfmfiles"
using namespace sfmf;
#include "SfMFiles/FeatureDescriptors.hpp"
#include "../io.hpp"

int
test1(int argc, char **argv)
{
    std::string siftFName = argv[0];
    LOG_INFO("Will load descriptors from file " << siftFName);

    std::vector<SIFTFeature> sift;
    loadSIFT(siftFName.c_str(), sift);

    LOG_EXPR(sift.size());
    std::cout << sift[0] << std::endl;

    return EXIT_SUCCESS;
}

int
main(int argc, char **argv)
{
    cmdc::Logger::setLogLevels(cmdc::LOGLEVEL_DEBUG);

    if(argc == 1) {
        std::cout << "Usage:\n\t" << argv[0] << "<in: testNum>" << std::endl;
        return EXIT_FAILURE;
    }

    int testNum = atoi(argv[1]);
    switch(testNum) {
    case 1:
        return test1(argc - 2, &argv[2]);
        break;
    default:
        LOG_WARN("No test " << testNum);
        return EXIT_FAILURE;
    }


    return EXIT_SUCCESS;
}
#undef NDEBUG

#include <SfMFiles/sfmfiles>
using namespace sfmf;
#include "../io.hpp"

int
test1(int argc, char **argv)
{
    CompressedFileReader f("/tmp/bla.gz");
    int x, y, z;
    f >> x >> y >> z;
    LOG_EXPR(x);
    LOG_EXPR(y);
    LOG_EXPR(z);
}

int
main(int argc, char **argv)
{
    cmdc::Logger::setLogLevels(cmdc::LOGLEVEL_DEBUG);

    if(argc == 1) {
        std::cout << "Usage:\n\t" << argv[0] << "<in: testNum> <in: option.patch> <in: bundle.out> <in: imWidth> <in: imHeight>" << std::endl;
        return EXIT_FAILURE;
    }

    int testNum = atoi(argv[1]);
    switch(testNum) {
    case 1:
        return test1(argc - 1, &argv[1]);
        break;
    default:
        LOG_ERROR("Test case " << testNum << " not recognized");
        return EXIT_FAILURE;
    }


    return EXIT_SUCCESS;
}
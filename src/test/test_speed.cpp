#include <SfMFiles/sfmfiles>
#include <iostream>

#include <Eigen/Dense>

const int nPnts = 100000000;

void
test1_matprod_double()
{
    LOG_INFO(__PRETTY_FUNCTION__);
    Eigen::MatrixX4d pnts = Eigen::MatrixX4d::Random(nPnts, 4);
    Eigen::MatrixXd P = Eigen::MatrixXd::Random(4, 3);

    Eigen::MatrixX3d projPnts = pnts * P.transpose();
}

void
test2_matprod_float()
{
    LOG_INFO(__PRETTY_FUNCTION__);
    Eigen::MatrixX4f pnts = Eigen::MatrixX4f::Random(nPnts, 4);
    Eigen::MatrixXf P = Eigen::MatrixXf::Random(4, 3);

    Eigen::MatrixX3f projPnts = pnts * P.transpose();
}

void
test3_forloop_double()
{
    LOG_INFO(__PRETTY_FUNCTION__);
    Eigen::MatrixX4d pnts = Eigen::MatrixX4d::Random(nPnts, 4);
    Eigen::MatrixXd P = Eigen::MatrixXd::Random(4, 3);

    Eigen::MatrixX3d projPnts(nPnts, 3);
    for(int i = 0; i < nPnts; i++) {
        projPnts.row(i) = P * projPnts.row(0);
    }
}

void
test4_forloop_float()
{
    LOG_INFO(__PRETTY_FUNCTION__);
    Eigen::MatrixX4f pnts = Eigen::MatrixX4f::Random(nPnts, 4);
    Eigen::MatrixXf P = Eigen::MatrixXf::Random(4, 3);

    Eigen::MatrixX3f projPnts(nPnts, 3);
    for(int i = 0; i < nPnts; i++) {
        projPnts.row(i) = P * projPnts.row(0);
    }
}

int
main(int argc, char const* argv[])
{

    cmdc::Logger::setLogLevels(cmdc::LOGLEVEL_DEBUG);

    int testCase = atoi(argv[1]);

    switch(testCase) {
    case 1:
        test1_matprod_double();
        break;
    case 2:
        test2_matprod_float();
        break;
    case 3:
        test3_forloop_double();
        break;
    case 4:
        test4_forloop_float();
        break;
    default:
        LOG_ERROR("Unknown test case " << testCase);
    }


    return EXIT_SUCCESS;
}
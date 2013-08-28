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

#include <iostream>

int
test1(int argc, char** argv)
{
    LOG_INFO("Project 3D points from bundler reconstruction");

    using namespace BDATA;
    using namespace std;

    const char* patchFName = argv[1];
    const char* bundlerFName = argv[2];
    int width = atoi(argv[3]);
    int height = atoi(argv[4]);
    LOG_EXPR(width);
    LOG_EXPR(height);

    PMVS::PMVSData pmvsData(patchFName, false);
    BDATA::BundlerData bundlerData(bundlerFName, false);

    pmvsData.loadCamerasAndImageFilenames("", false);

    LOG_EXPR(pmvsData.getNPatches());
    LOG_EXPR(pmvsData.getNCameras());

    BDATA::Camera::Vector::iterator cam = bundlerData.getCameras().begin();
    for(; cam != bundlerData.getCameras().end(); cam++) {
    }

    BDATA::PointInfo::Vector::iterator pnt = bundlerData.getPointInfo().begin();
    for(; pnt != bundlerData.getPointInfo().end(); pnt++) {
        Eigen::Vector3d w = pnt->position;


        for(BDATA::PointEntry::Vector::iterator pEnt = pnt->viewList.begin(); pEnt != pnt->viewList.end(); pEnt++) {
            BDATA::Camera& camB = bundlerData.getCameras()[pEnt->camera];
            PMVS::Camera& camP = pmvsData.getCameras()[pEnt->camera];

            if(camP.isValid() == false) continue;

            Eigen::Vector2d imP, imB, keyPos;
            keyPos = pEnt->keyPosition;

            keyPos[1] = height - (height / 2 + keyPos[1]);
            keyPos[0] = keyPos[0] + width / 2.0;

            camP.world2im(w, imP);

            camB.world2imPmvs(w, imB, false, width, height);
            //imB[1] = height - imB[1];

            if(isnan(imP[0]) || isnan(imP[1])) {
                LOG_EXPR(camP);
            }

            char msg[1000];
            sprintf(msg, "key = [%10.5f, %10.5f], bun = [%10.5f, %10.5f], pmvs = [%10.5f, %10.5f]\n",
                    keyPos[0], keyPos[1], imB[0], imB[1], imP[0], imP[1]);
            LOG_INFO(msg);

        }
    }

    return EXIT_SUCCESS;
}

int
test2(int argc, char** argv)
{
    LOG_INFO("Project 3D points from PMVS reconstruction");

    using namespace BDATA;
    using namespace std;

    const char* patchFName = argv[1];
    const char* bundlerFName = argv[2];
    int width = atoi(argv[3]);
    int height = atoi(argv[4]);
    LOG_EXPR(width);
    LOG_EXPR(height);

    PMVS::PMVSData pmvsData(patchFName, false);
    BDATA::BundlerData bundlerData(bundlerFName, false);

    pmvsData.loadCamerasAndImageFilenames();

    LOG_EXPR(pmvsData.getNPatches());
    LOG_EXPR(pmvsData.getNCameras());

    LOG_INFO("Verifying if points that are seen by camera project into image");

    PMVS::Patch::Vector::iterator patch = pmvsData.getPatches().begin();
    for(int patchIdx = 0; patch != pmvsData.getPatches().end(); patch++, patchIdx++) {
        //LOG_EXPR(patchIdx);

        vector<uint32_t> camIdxs = patch->goodCameras;
        //camIdxs.insert(camIdxs.end(), patch->badCameras.begin(), patch->badCameras.end());

        for(vector<uint32_t>::iterator camIdx = camIdxs.begin(); camIdx != camIdxs.end(); camIdx++) {
            PMVS::Camera& pmvsCam = pmvsData.getCameras()[*camIdx];
            BDATA::Camera& bundlerCam = bundlerData.getCameras()[*camIdx];

            Eigen::Vector3d w(patch->position[0], patch->position[1], patch->position[2]);
            Eigen::Vector2d im, imB;
            pmvsCam.world2im(w, im);

            bundlerCam.world2imPmvs(w, imB, false, width, height);
            //imB[1] = height - imB[1];

            if(im[0] < 0 || im[0] >= width || im[1] < 0 || im[1] >= height) {
                printf("[sfmfiles] w = [%10.6f, %10.6f, %10.6f] -> im = [%10.6f, %10.6f] (pmvs), im = [%10.6f, %10.6f] (bundler)\n", w[0], w[1], w[2], im[0], im[1], imB[0], imB[1]);
            }

        }
    }

    return EXIT_SUCCESS;
}


int
main(int argc, char** argv)
{
    cmdc::init();
    if(argc == 1) {
        std::cout << "Usage:\n\t" << argv[0] << "<in: testNum> <in: option.patch> <in: bundle.out> <in: imWidth> <in: imHeight>" << std::endl;
        return EXIT_FAILURE;
    }

    int testNum = atoi(argv[1]);
    switch(testNum) {
    case 1:
        return test1(argc - 1, &argv[1]);
        break;
    case 2:
        return test2(argc - 1, &argv[1]);
        break;
    default:
        LOG_ERROR("Test case " << testNum << " not recognized");
        return EXIT_FAILURE;
    }

    cmdc::deinit();
    return EXIT_SUCCESS;
}
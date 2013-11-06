#include "mex.h"

// Reading PMVS and Bundler files
#include "SfMFiles/sfmfiles"
using namespace sfmf;

void
packCameras(const Bundler::Reconstruction &bundle, mxArray **cams)
{
    assert((*cams) == NULL);
    *cams = mxCreateCellMatrix(bundle.getNCameras(), 1);

    const char *fieldNames[5] = {"translation", "rotation", "focal_length", "k1", "k2"};

    for (int camIdx = 0; camIdx < bundle.getNCameras(); camIdx++) {
        const Bundler::Camera &cam = bundle.getCameras()[camIdx];
        mxArray *camStruct = mxCreateStructMatrix(1, 1, 5, fieldNames);

        // Rotation
        mxArray *rotMx = mxCreateDoubleMatrix(3, 3, mxREAL);
        double *rot = mxGetPr(rotMx);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rot[j * 3 + i] = cam.rotation(i, j);
            }
        }
        mxSetField(camStruct, 0, "rotation", rotMx);

        // Translation
        mxArray *transMx = mxCreateDoubleMatrix(3, 1, mxREAL);
        double *trans = mxGetPr(transMx);
        for (int i = 0; i < 3; i++) {
            trans[i] = cam.translation(i);
        }
        mxSetField(camStruct, 0, "translation", transMx);

        // Focal Length
        mxArray *focalLenMx = mxCreateDoubleMatrix(1, 1, mxREAL);
        mxGetPr(focalLenMx)[0] = cam.focalLength;
        mxSetField(camStruct, 0, "focal_length", focalLenMx);

        // K1
        mxArray *k1Mx = mxCreateDoubleMatrix(1, 1, mxREAL);
        mxGetPr(k1Mx)[0] = cam.k1;
        mxSetField(camStruct, 0, "k1", k1Mx);

        // K2
        mxArray *k2Mx = mxCreateDoubleMatrix(1, 1, mxREAL);
        mxGetPr(k2Mx)[0] = cam.k2;
        mxSetField(camStruct, 0, "k2", k2Mx);

        // Put the struct in the cell array
        mxSetCell(*cams, camIdx, camStruct);
    }
}

// INPUT:
//   - Bundle filename
// OUTPUT:
//   - Cameras

void
mexFunction(int nlhs, mxArray *plhs[],
            int nrhs, const mxArray *prhs[])
{
    cmdc::Logger::setLogLevels(cmdc::LOGLEVEL_NOPRINTING);

    if(nrhs != 1) mexErrMsgTxt("Expecting 1 input");
    if(nlhs != 1) mexErrMsgTxt("Expecting 1 output");

    char *bundleFName = mxArrayToString(prhs[0]);
    if(bundleFName != NULL) {
        Bundler::Reconstruction bundle(bundleFName);
        mxFree(bundleFName);
        bundleFName = NULL;

        mxArray *cams = NULL;
        packCameras(bundle, &cams);

        plhs[0] = cams;
    }
}
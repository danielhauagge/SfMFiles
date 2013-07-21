#define __STDC_FORMAT_MACROS
#include <time.h>
#include <math.h>
#include <inttypes.h>
#include <set>

#include <SfMFiles/sfmfiles>

using namespace std;

typedef struct {
    char name[256];
    int w, h;
} sizeentry;

static inline
double
degrees(double a)
{
    return(a * 360.0 / 2.0 / M_PI);
}

static vector<sizeentry>
read_sizes(const char* fname)
{
    FILE* fp = fopen(fname, "r");
    sizeentry se;
    vector<sizeentry> sizelist;
    char fmt[256];
    sprintf(fmt, "%%%zds %%d %%d\n", sizeof(se.name));
    while(fscanf(fp, fmt, se.name, &se.w, &se.h) == 3) {
        se.name[sizeof(se.name) - 1] = 0;
        //printf("%s %d %d\n",se.name,se.w,se.h);
        sizelist.push_back(se);
    }
    fclose(fp);
    return(sizelist);
}

void
loadSelectedCameras(const char* fname, std::set<int>& camIdxs)
{
    std::ifstream f(fname);
    assert(f.good());

    //camIdxs.resize(0);
    while(f.good()) {
        int camIdx;
        f >> camIdx;
        if(!f.good()) return;
        camIdxs.insert(camIdx);
    }
}

int
main(int argc, const char* argv[])
{
    using namespace BDATA;

    int maxCamerasPerPoint = 1024;

    if(argc == 1) {
        std::cout << "Usage:\n\t" << argv[0] << " <in:bundle.out> <in:list.txt> <in:list.sizes.txt> <out:visibility.camviz> [<in:selected_cams.txt>]" << std::endl;
        std::cout << "\nselected_cams.txt: a file containing the indexes of the cameras the user wishes to use, only\n"
                  <<   "\tthese cameras will be considered for visibility expansion." << std::endl;
        return EXIT_FAILURE;
    }

    const char* bundleFName = argv[1];
    const char* listFName = argv[2];
    const char* sizes_fname = argv[3];
    const char* visibilityListFName = argv[4];
    const char* selectedCamerasFName = (argc == 6) ? argv[5] : NULL;

    std::set<int> selectedCams;
    if(selectedCamerasFName != NULL) {
        PRINT_MSG("Loading selected cameras from " << selectedCamerasFName);
        loadSelectedCameras(selectedCamerasFName, selectedCams);
    }

    PRINT_MSG("Loading bundle file");
    BDATA::BundlerData bundler(bundleFName);
    PRINT_EXPR(bundler.getNCameras());
    PRINT_EXPR(bundler.getNValidCameras());
    PRINT_EXPR(bundler.getNPoints());

    if(selectedCamerasFName == NULL) {
        //selectedCams.resize(bundler.getNCameras());
        for(int i = 0; i < selectedCams.size(); i++) {
            selectedCams.insert(i);
        }
    }

    PRINT_MSG("Building camera to point index");
    bundler.buildCam2PointIndex();

    try {
        PRINT_MSG("Loading list file");
        bundler.readListFile(listFName);
    } catch (sfmf::Error e) {
        PRINT_MSG("ERROR: Caught exception");
        PRINT_MSG(" WHAT: " << e.what());
    }

    // Test transforms
    PointInfo& pntInfo = bundler.getPointInfo()[0];

    PRINT_EXPR(pntInfo.position.transpose());
    const int pntIdx = 0;
    Eigen::Vector2d featPos = pntInfo.viewList[pntIdx].keyPosition;

    vector<sizeentry> sizelist = read_sizes(sizes_fname);
    {
        int n = sizelist.size();
        printf("sizelist: Read %lu entries.\n", sizelist.size());
        //printf("%s %d %d\n",sizelist[0].name,sizelist[0].w,sizelist[0].h);
        //printf("%s %d %d\n",sizelist[n-1].name,sizelist[n-1].w,sizelist[n-1].h);
    }
    {
        Eigen::Vector3d ctr_ww;
        int totalnvc = 0;
        int totalnvc2 = 0;
        int np = bundler.getNPoints();
        const Camera::Vector& cam = bundler.getCameras();
        int nc = cam.size();
        for(int p = 0; p < np; p++) {
            PointInfo& pi = bundler.getPointInfo()[p];
            Eigen::Vector3d p_ww = pi.position;
            int nvc = pi.viewList.size();
            {
                //if(p<1) printf("point %d has %d cameras\n",p,nvc);
                totalnvc += nvc;
                for(int vc = 0; vc < nvc; vc++) {
                    int cid = pi.viewList[vc].camera;
                    if(cam[cid].isValid()) totalnvc2++;
                }
                //if(p<1) printf("pos_ww=(%g,%g,%g) nvc=%d\n",p_ww(0),p_ww(1),p_ww(2),nvc);
            }
            ctr_ww = ctr_ww + p_ww;
        }
        ctr_ww = ctr_ww / np;
        //printf( "ctr_ww=(%g,%g,%g) np=%d\n",ctr_ww(0),ctr_ww(1),ctr_ww(2),np);
        printf("average %g cameras per point\n", (double) totalnvc / np);
        printf("average %g valid cameras per point\n", (double) totalnvc2 / np);

        // visibility cone expansion
        // T=center(boundingvolume(t(x) for x in viewList))
        // minV=min(T . t(x) for x in viewList)
        // newviewlist=(x for x in Camera if T . t(x)>=minV)

        // rho is proportional to the spatial noise of the dataset
        // calculations will be performed on the rho-th percentile
        // increasing rho will reduce the number of cameras added
        double rho = 0.10;
        vector<Eigen::Vector3d> cam_c_ww;
        for(int cid = 0; cid < nc; cid++) {
            if(!cam[cid].isValid()) {
                cam_c_ww.push_back(Eigen::Vector3d(0, 0, 0));
            } else {
                Eigen::Vector3d c_ww;
                cam[cid].cam2world(Eigen::Vector3d(0, 0, 0), c_ww);
                cam_c_ww.push_back(c_ww);
            }
        }
        printf("Calculating T ...\n");
        vector<Eigen::Vector3d> Tlist;
        for(int p = 0; p < np; p++) {
            PointInfo& pi = bundler.getPointInfo()[p];
            Eigen::Vector3d p_ww = pi.position;
            int nvc = pi.viewList.size();
            vector<double> xcoord, ycoord, zcoord;
            int m = max(min(1, nvc / 300), 10);

            for(int vc = 0; vc < nvc; vc++) {
                if((vc % m) != 0) continue;
                int cid = pi.viewList[vc].camera;
                if(!cam[cid].isValid()) continue;
                Eigen::Vector3d& c_ww = cam_c_ww[cid];
                Eigen::Vector3d t_w = (c_ww - p_ww).normalized();
                xcoord.push_back(t_w(0));
                ycoord.push_back(t_w(1));
                zcoord.push_back(t_w(2));
            }

            // approximate center of bounding volume is center of bounding box of middle 90th percentile
            sort(xcoord.begin(), xcoord.end());
            sort(ycoord.begin(), ycoord.end());
            sort(zcoord.begin(), zcoord.end());
            int a = int(xcoord.size() * (rho / 2.0) + 0.5), b = int(xcoord.size() * (1.0 - rho / 2.0) + 0.5);
            Eigen::Vector3d bbcenter_w((xcoord[a] + xcoord[b]) / 2.0, (ycoord[a] + ycoord[b]) / 2.0, (zcoord[a] + zcoord[b]) / 2.0);
            Tlist.push_back(bbcenter_w.normalized());
        }


        PRINT_MSG("Calculating minV ...");
        vector<double> minVlist;
        for(int p = 0; p < np; p++) {
            PointInfo& pi = bundler.getPointInfo()[p];
            Eigen::Vector3d p_ww = pi.position;
            int nvc = pi.viewList.size();
            Eigen::Vector3d& T_w = Tlist[p];
            vector<double> minvlist;
            int m = min(max(1, nvc / 300), 10);
            for(int vc = 0; vc < nvc; vc++) {
                if((vc % m) != 0) continue;
                int cid = pi.viewList[vc].camera;
                if(!cam[cid].isValid()) continue;
                Eigen::Vector3d& c_ww = cam_c_ww[cid];
                Eigen::Vector3d t_w = (c_ww - p_ww).normalized();
                double minv = T_w.dot(t_w);
                minvlist.push_back(minv);
            }
            sort(minvlist.begin(), minvlist.end());
            double minV = minvlist[int(minvlist.size() * rho + 0.5)];
            minVlist.push_back(minV);
            if((p % 2000) == 0) {
                Eigen::Vector3d p_wc = p_ww - ctr_ww;
                printf("pos_wc=(%g,%g,%g) cone.direction=(%g,%g,%g) cone.halfangle=%g deg (%g deg)\n", p_wc(0), p_wc(1), p_wc(2), T_w(0), T_w(1), T_w(2),
                       degrees(acos(minV)), degrees(acos(minvlist[0])));
            }
        }

        PRINT_MSG("Calculating viewList ...");
        int64_t work_units = (int64_t) np * nc;
        int64_t work_done = 0;
        double work_t0 = time(0);
        double rlprint_t0 = 0, rlprint_dt = 15;
        unsigned char* is_original = new unsigned char[nc];
        //const char *oname = visibilityListFName;//"expansion.dat";
        FILE* fp = fopen(visibilityListFName, "wb");
        if(fp == 0) {
            perror(visibilityListFName);
            exit(EXIT_FAILURE);
        }
        fprintf(fp, "#camviz\n%d %d\n", np, maxCamerasPerPoint);


        for(int p = 0; p < np; p++) {
            if(p % 100 == 0) printf("%6d/%d points processed\n", p, np);

            PointInfo& pi = bundler.getPointInfo()[p];
            Eigen::Vector3d p_ww = pi.position;
            int nvc = pi.viewList.size();
            Eigen::Vector3d& T_w = Tlist[p];
            double minV = minVlist[p];
            PointEntry::Vector expansion;

            for(int cid = 0; cid < nc; cid++) {
                is_original[cid] = 0;
            }

            for(int vc = 0; vc < nvc; vc++) {
                int cid = pi.viewList[vc].camera;
                if(!cam[cid].isValid()) continue;
                is_original[cid] = 1;
            }

            int ncull = 0;
            for(int cid = 0; cid < nc && expansion.size() < maxCamerasPerPoint; cid++) {
                work_done = work_done + 1;
                if((cid % 100) == 0) {
                    double t1 = time(0);
                    if(t1 - rlprint_t0 > rlprint_dt) {
                        rlprint_t0 = t1;
                        //printf("%" PRId64 "/%" PRId64 ", %g min remaining\n", work_done, work_units, ((double)work_units/(double)work_done-1)*(t1-work_t0)/60.0);
                        fflush(0);
                    }
                }

                if(!cam[cid].isValid()) continue;
                if(selectedCamerasFName != NULL && selectedCams.count(cid) == 0) continue;

                Eigen::Vector3d& c_ww = cam_c_ww[cid];
                Eigen::Vector3d t_w = (c_ww - p_ww).normalized();
                double x = T_w.dot(t_w);
                if(T_w.dot(t_w) >= minV || is_original[cid]) {
                    // contraction by visibility
                    // verify that the point projets into the image.
                    sizeentry& se = sizelist[cid];
                    if(se.w > 0 && se.h > 0) {
                        Eigen::Vector2d p_i;
                        cam[cid].world2im(p_ww, p_i, true, se.w, se.h);
                        //if(p<1 && cid<1000000) printf("pos_i%d = (%g,%g)pixels (%g,%g)texc\n",cid,p_i(0),p_i(1),(p_i(0)+0.5)/se.w,(p_i(1)+0.5)/se.h);
                        // NOTE: these texture coordinates are upper-left
                        double u = (p_i(0) + 0.5) / se.w;
                        double v = (p_i(1) + 0.5) / se.h;
                        if(u >= 0 && u <= 1 && v >= 0 && v <= 1) {
                            PointEntry pe(cid, -1, Eigen::Vector2d(0, 0));
                            expansion.push_back(pe);
                        } else {
                            ncull = ncull + 1;
                        }
                    }
                }
            }
            //if((p % 100)==0) printf("%d became %d (%d culled)\n", nvc, expansion.size(), ncull);
            if(0) pi.viewList = expansion;
            if(1) {
                // >>> struct.unpack('<1000i',f.read(4000))
                int ne = expansion.size();
                int nemit = 0;
                //int m = max(1, ne / maxCamerasPerPoint);
                for(int e = 0; e < ne && nemit < maxCamerasPerPoint; e++) {
                    //if(e % m) continue;
                    int32_t cid = expansion[e].camera;
                    if(fwrite(&cid, sizeof(cid), 1, fp) != 1) {
                        perror(visibilityListFName);
                        exit(EXIT_FAILURE);
                    }
                    nemit++;
                }
                for(int e = nemit; e < maxCamerasPerPoint; e++) {
                    int32_t pad = -1;
                    if(fwrite(&pad, sizeof(pad), 1, fp) != 1) {
                        perror(visibilityListFName);
                        exit(EXIT_FAILURE);
                    }
                }
            }
            //if(p>200) break;
        }
        fclose(fp);
        delete[] is_original;
        if(0) {
            int totalnvc3 = 0;
            for(int p = 0; p < np; p++) {
                PointInfo& pi = bundler.getPointInfo()[p];
                int nvc = pi.viewList.size();
                totalnvc3 += nvc;
            }
            printf("average %g cameras per point (originally %g)\n", (double)totalnvc3 / np, (double)totalnvc / np);
            printf("Writing expanded bundle ...\n");
            bundler.writeFile("expanded.bundle");
        }
    }

    printf("Done.\n");
    return(EXIT_SUCCESS);
}


#include <iostream>
#include <cstdio>

#include <OptParser/optparser>

#include <Macros.hpp>
#include <PMVSData.hpp>
#include <BundlerData.hpp>

int
main(int argc, const char* argv[])
{
  //BDATA::PointInfo::Vector x(1000);
  //x.resize(0);


  using namespace BDATA;

  std::vector<std::string> args;
  std::map<std::string, std::string> opts;

  OptionParser optParser(&args, &opts);
  optParser.addUsage("[OPTIONS] <in:model.patch> <in:model.out> <out:model.out>");
  optParser.addDescription("Utility for converting from PMVS's .patch file format to the bundler file format");
  optParser.addFlag("dontLoadOption", "-p", "--dont-load-options", "Do not try to load options file for the PMVS file (in case PMVS cameras"
		    " indexes do not map to Bundler cameras)");        
  optParser.addFlag("includeBadCameras", "-b", "--include-bad-cameras", "Include in the output bundle file the PMVS cameras that were labeled as bad.");      
  optParser.parse(argc, argv);

  std::string pmvsFName = args[0];
  std::string inBundleFName = args[1];
  std::string outBundleFName = args[2];
	
  bool tryLoadOptions = !atoi(opts["dontLoadOption"].c_str());
  bool includeBadCameras = atoi(opts["includeBadCameras"].c_str());

  PRINT_EXPR(outBundleFName);
  PRINT_EXPR(inBundleFName);
  PRINT_EXPR(pmvsFName);

  PRINT_MSG("Loading the bundle file");
  BDATA::BundlerData::Ptr bundle = BDATA::BundlerData::New(inBundleFName.c_str());

  //PRINT_MSG("Loading the pmvs file");
  //PMVS::PMVSData::Ptr pmvs = PMVS::PMVSData::New(pmvsFName.c_str(), tryLoadOptions);
  //PRINT_MSG("Done loading stuff");


  BDATA::PointInfo::Vector x(1000);
#if 0
  //x.resize(0);

  //bundle->getPointInfo().resize(0);


  PMVS::Patch::Vector::iterator patch = pmvs->getPatches().begin();

  PRINT_MSG("Adding patches to bundle file");
  for(int i = 0; i < pmvs->getNPatches(); i++, patch++) {
    BDATA::PointInfo pinfo;

    // Position
    for(int j = 0; j < 3; j++) pinfo.position(j) = patch->position(j);

    // Color
    pinfo.color.r = (unsigned char) patch->color(0) * 255;
    pinfo.color.g = (unsigned char) patch->color(1) * 255;
    pinfo.color.b = (unsigned char) patch->color(2) * 255;

    // Cameras
    int nCams = patch->goodCameras.size();
    if(includeBadCameras) nCams += patch->badCameras.size();
    pinfo.viewList.resize(nCams);
    
    int camIdx = 0;
    for(int j = 0; j < patch->goodCameras.size(); j++, camIdx++) {
      pinfo.viewList[camIdx] = BDATA::PointEntry(patch->goodCameras[j], -1, Eigen::Vector2d());
    }
    
    if(includeBadCameras) {
      for(int j = 0; j < patch->badCameras.size(); j++, camIdx++) {
	pinfo.viewList[camIdx] = BDATA::PointEntry(patch->badCameras[j], -1, Eigen::Vector2d());
      }
    }

    bundle->getPointInfo().push_back(pinfo);
  }

  PRINT_MSG("Writing " << outBundleFName);
  bundle->writeFile(outBundleFName.c_str());
#endif
  return EXIT_SUCCESS;
}

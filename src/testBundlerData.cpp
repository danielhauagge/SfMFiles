#include "BundlerData.hpp"
#include "Macros.hpp"

int
main(int argc, const char* argv[])
{
  if(argc == 1) {
    std::cout << "Usage:\n\t" << argv[0] << " <bundle.out> <list.txt> <cam index>" << std::endl;
    return EXIT_FAILURE;
  }

  const char* bundleFName = argv[1];
  const char* listFName = argv[2];
  int camNum = atoi(argv[3]);
  
  LOG("Loading bundle file");
  BDATA::BundlerData bundler(bundleFName);
  PRINT_VAR(bundler.getNCameras());
  PRINT_VAR(bundler.getNValidCameras());
  PRINT_VAR(bundler.getNPoints());

  LOG("Building camera to point index");
  bundler.buildCam2PointIndex();

  try {
    LOG("Loading list file");
    bundler.loadListFile(listFName);
  } catch (BDATA::BadFileException e) {
    LOG("Caught exception");
    LOG("What: " << e.what());
  }  

  PRINT_VAR(bundler.getListFileName());
  PRINT_VAR(bundler.getImageFileNames()[camNum]);

  return EXIT_SUCCESS;
}

#include "BundlerData.hpp"
#include "Macros.hpp"

int
main(int argc, const char* argv[])
{
  const char* bundleFName = argv[1];
  const char* listFName = argv[2];
  int camNum = atoi(argv[3]);
  
  LOG("Loading bundle file");
  BDATA::BundlerData bundler(bundleFName);
  PRINT_VAR(bundler.getNCameras());
  PRINT_VAR(bundler.getNValidCameras());
  PRINT_VAR(bundler.getNPoints());

  try {
    LOG("Loading list file");
    bundler.loadListFile(listFName);
  } catch (BDATA::BadFileException e) {
    LOG("Caught exception");
    LOG("What: " << e.what());
  }  

  PRINT_VAR(bundler.getListFileName());
  PRINT_VAR(bundler.getImageFileName(camNum));

  return EXIT_SUCCESS;
}

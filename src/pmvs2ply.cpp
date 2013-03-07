#include <sfmfiles>

#include <OptParser/optparser>

#include <iostream>
#include <cstdio>

int
main(int argc, const char* argv[])
{
  using namespace BDATA;

  std::vector<std::string> args;
  std::map<std::string, std::string> opts;

  OptionParser optParser(&args, &opts);
  optParser.addUsage("[OPTIONS] <in:model.patch> <out:model.ply>");
  optParser.addDescription("Utility for converting from PMVS's .patch file format to a .ply");
  optParser.addFlag("dontLoadOption", "-p", "--dont-load-options", "Do not try to load options file for the reconstruction (used to remap camera indexes)");      
  optParser.parse(argc, argv);

  std::string pmvsFName = args[0];
  std::string plyFName = args[1];
	
  bool tryLoadOptions = !atoi(opts["dontLoadOption"].c_str());

  PMVS::PMVSData pmvs(pmvsFName.c_str(), tryLoadOptions);

  // Write PLY header
  FILE* plyF = fopen(plyFName.c_str(), "wb");
  if(plyF == NULL) {
    std::cout << "ERROR: Could not open file " << plyFName << " for writing" << std::endl;
    return EXIT_FAILURE;
  }
  fprintf(plyF, "ply\n");
  fprintf(plyF, "format binary_little_endian 1.0\n");
  fprintf(plyF, "element vertex %d\n", pmvs.getNPatches());
  fprintf(plyF, "property float x\nproperty float y\nproperty float z\n");
  fprintf(plyF, "property float red\nproperty float green\nproperty float blue\nend_header\n");

  PRINT_MSG("Processing data");
  PMVS::Patch::Vector::iterator patch = pmvs.getPatches().begin();;
  for(int i = 0; i < pmvs.getNPatches(); i++, patch++) {
    for(int j = 0; j < 3; j++) {
      float coord = patch->position[j];
      fwrite(&coord, sizeof(float), 1, plyF);
      //fprintf(plyF, "%f ", coord);
    }

    for(int j = 0; j < 3; j++) {
      float color = patch->color[j];
      //LOG(color);
      fwrite(&color, sizeof(float), 1, plyF);
    }

    //fprintf(plyF, "\n");
  }

  fclose(plyF);

  return EXIT_SUCCESS;
}

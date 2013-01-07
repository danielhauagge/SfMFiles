#include <Macros.hpp>
#include <PMVSData.hpp>

#include <iostream>

int
main(int argc, char** argv)
{
  using namespace BDATA;

  if(argc == 1) {
    std::cout << "Usage:\n\t" << argv[0] << " <in:pmvs.patch> <in:frac> <out:subsampled_pmsvs.patch>" << std::endl;
    return EXIT_FAILURE;
  }

  const char* pmvsFName = argv[1];
  float frac = atof(argv[2]);
  const char* sampledPmvsFName = argv[3];

  PMVS::PMVSData pmvs(pmvsFName);
  PMVS::PMVSData pmvsSamp;

  PMVS::Patch::Vector& patchSamp = pmvsSamp.getPatches();
  for(int i = 0; i < pmvs.getNPatches(); i++) {
    double prob = rand() / (RAND_MAX + 1.0);

    if(prob < frac) {
      patchSamp.push_back(pmvs.getPatches()[i]);
    }
  }
  
  pmvsSamp.writeFile(sampledPmvsFName);

  return EXIT_SUCCESS;
}

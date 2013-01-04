#include <Macros.hpp>
#include <PMVSData.hpp>

#include <iostream>

int
main(int argc, const char** argv)
{
	using namespace BDATA::PMVS;

	if(argc == 1) {
		std::cout << "Usage:\n\t" << argv[0] << " <out:merged.patch> <in:file1.patch> <in:file2.patch> ..." << std::endl;
		return EXIT_FAILURE;
	}

	const char* outFName = argv[1];

	const char** inFNames = &argv[2];
	int nFiles = argc - 2;

	PRINT_VAR(nFiles);

	PMVSData merged;

	for(int i = 0; i < nFiles; i++) {
		PMVSData pmvs(inFNames[i]);

		merged.mergeWith(pmvs);
	}

	LOG("Writing file " << outFName);
	merged.writeFile(outFName);

	return EXIT_SUCCESS;
}
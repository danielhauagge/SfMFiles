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

	PRINT_EXPR(nFiles);

	PMVSData merged;

	for(int i = 0; i < nFiles; i++) {
		PMVSData pmvs(inFNames[i]);

		merged.mergeWith(pmvs);
	}

	PRINT_MSG("Writing file " << outFName);
	merged.writeFile(outFName);

	return EXIT_SUCCESS;
}

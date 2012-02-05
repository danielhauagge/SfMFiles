// Copyright (C) 2011 by Daniel Cabrini Hauagge
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

#include <Macros.hpp>
#include <BundlerData.hpp>

int
main(int argc, char **argv)
{
    if (argc == 1) {
        std::cout << "Usage:\n\t" << argv[0] << " <in:bunle.out> <b|a> <out:bundle.out>" << std::endl;
        return EXIT_FAILURE;
    }
    
    const char *inBundleFName = argv[1];
    const char *outMode = (argc > 2)?argv[2]:NULL;
    const char *outBundleFName = (argc > 2)?argv[3]:NULL;
    
    LOG("Loading data");
    BDATA::BundlerData bundle(inBundleFName);
    
    if (outMode == NULL)
        return EXIT_SUCCESS;
    
    if(strcmp(outMode, "b") == 0) {
        LOG("Writing binary file");
        bundle.writeFile(outBundleFName, false);
    } else if(strcmp(outMode, "a") == 0) {
        LOG("Writing ASCII file");
        bundle.writeFile(outBundleFName, true);
    } else {
        LOG("ERROR: Unrecognized mode " << outMode << ". Run command with no argments for usage");
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}
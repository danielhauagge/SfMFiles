#include <PMVSData.hpp>

#include <iostream>

int
main(int argc, char **argv)
{
    const char *patchFName = argv[1];
    
    BDATA::PMVS::PMVSData pmvsData(patchFName);
    
    return EXIT_SUCCESS;
}
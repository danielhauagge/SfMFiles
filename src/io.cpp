#include "io.hpp"

#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/device/file.hpp>

SFMFILES_NAMESPACE_BEGIN

CompressedFileReader::CompressedFileReader(const char *fname, bool throwException)
{
    FILE *file = fopen(fname, "rb");
    if(file == NULL) {
        if(throwException) {
            std::stringstream errMsg;
            errMsg << "Could not open " << fname << " for reading";
            throw sfmf::Error(errMsg.str().c_str());
        } else return;
    }

    // Read the first two bites
    char fileType[3];
    fread(fileType, sizeof(char), sizeof(fileType) - 1, file);
    fileType[2] = '\0';
    fclose(file);

    // Is this a GZiped file?
    if (strcmp(fileType, "\x1f\x8b") == 0) {
        this->push(boost::iostreams::gzip_decompressor());
    }

    this->push(boost::iostreams::file_source(fname));
    if(!this->good()) {
        if(throwException) {
            std::stringstream err;
            err << "Could not read file " << fname;
            throw sfmf::Error(err.str());
        } else return;
    }
}

SFMFILES_NAMESPACE_END
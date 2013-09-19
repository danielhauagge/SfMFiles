#include "SfMFiles/FeatureDescriptors.hpp"
#include "io.hpp"

int
loadSIFT(const char* fname, std::vector<SIFTFeature>& features, bool throwException)
{
    CompressedFileReader f(fname, throwException);

    if(!f.good()) {
        if(throwException) {
            std::stringstream errMsg;
            errMsg << "Could not open file " << fname << " for reading";
            throw sfmf::IOError(errMsg.str());
        } else return 0;
    }

    const int SIFT_DIM = 128;

    int nDesc, dim;
    f >> nDesc >> dim;
    if(dim != SIFT_DIM) {
        if(throwException) {
            std::stringstream errMsg;
            errMsg << "Incompatible dimension for descriptor. Expecting " << SIFT_DIM << " but file desciptors are of size " << dim;
            throw sfmf::Error(errMsg.str());
        } else return 0;
    }

    features.resize(nDesc);
    std::vector<SIFTFeature>::iterator feat = features.begin();
    for (int i = 0; i < nDesc; i++, feat++) {
        f >> feat->x >> feat->y >> feat->scale >> feat->orientation;

        uint8_t* desc = feat->descriptor;
        for(int j = 0; j < SIFT_DIM; j++, desc++) {
            int v;
            f >> v;
            (*desc) = v;
        }
    }

    return 1;
}

std::ostream&
operator<<(std::ostream& s, const SIFTFeature& f)
{
    s << f.x << " " << f.y << " " << f.scale << " " << f.orientation << "\n";
    const uint8_t* desc = f.descriptor;

    for(int i = 0; i < sizeof(f.descriptor); i++, desc++) {
        s << (int)(*desc) << " ";
        if((i + 1) % 20 == 0) s << "\n";
    }

    return s;
}

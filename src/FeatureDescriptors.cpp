#include "SfMFiles/FeatureDescriptors.hpp"
#include "io.hpp"

void
loadSIFT(const char* fname, std::vector<SIFTFeature>& features)
{
    CompressedFileReader f(fname);

    const int SIFT_DIM = 128;

    int nDesc, dim;
    f >> nDesc >> dim;
    if(dim != SIFT_DIM) {
        std::stringstream errMsg;
        errMsg << "Incompatible dimension for descriptor. Expecting " << SIFT_DIM << " but file desciptors are of size " << dim;
        throw sfmf::Error(errMsg.str());
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
}

std::ostream&
operator<<(std::ostream& s, const SIFTFeature& f)
{
    LOG_EXPR(sizeof(f.descriptor));

    s << f.x << " " << f.y << " " << f.scale << " " << f.orientation << "\n";
    const uint8_t* desc = f.descriptor;

    for(int i = 0; i < sizeof(f.descriptor); i++, desc++) {
        s << (int)(*desc) << " ";
        if((i + 1) % 20 == 0) s << "\n";
    }

    return s;
}

#include "utils.hpp"
#include <arpa/inet.h>

// Ref: http://carnage-melon.tom7.org/stuff/jpegsize.html
/* portions derived from IJG code */

#define readbyte(a,b) do if(((a)=getc((b))) == EOF) return 0; while (0)
#define readword(a,b) do { int cc_=0,dd_=0; \
        if((cc_=getc((b))) == EOF \
                || (dd_=getc((b))) == EOF) return 0; \
        (a) = (cc_<<8) + (dd_); \
    } while(0)


int
scanhead(FILE* infile, int* image_width, int* image_height)
{
    int marker = 0;
    int dummy = 0;
    if ( getc(infile) != 0xFF || getc(infile) != 0xD8 )
        return 0;

    for (;;) {

        int discarded_bytes = 0;
        readbyte(marker, infile);
        while (marker != 0xFF) {
            discarded_bytes++;
            readbyte(marker, infile);
        }
        do readbyte(marker, infile);
        while (marker == 0xFF);

        if (discarded_bytes != 0) return 0;

        switch (marker) {
        case 0xC0:
        case 0xC1:
        case 0xC2:
        case 0xC3:
        case 0xC5:
        case 0xC6:
        case 0xC7:
        case 0xC9:
        case 0xCA:
        case 0xCB:
        case 0xCD:
        case 0xCE:
        case 0xCF: {
            readword(dummy, infile);	/* usual parameter length count */
            readbyte(dummy, infile);
            readword((*image_height), infile);
            readword((*image_width), infile);
            readbyte(dummy, infile);

            return 1;
            break;
        }
        case 0xDA:
        case 0xD9:
            return 0;
        default: {
            int length;

            readword(length, infile);

            if (length < 2)
                return 0;
            length -= 2;
            while (length > 0) {
                readbyte(dummy, infile);
                length--;
            }
        }
        break;
        }
    }
}

bool
hasExtension(std::string fname, const char* exts[])
{
    std::transform(fname.begin(), fname.end(), fname.begin(), ::tolower);

    for(int i = 0; exts[i] != NULL; i++) {
        if(strstr(fname.c_str(), exts[i]) != NULL) return true;
    }

    return false;
}

int
getImageSize(const char* fname, int& width, int& height, bool throwException)
{
    const char* jpgExts[]  = {".jpg", ".jpeg", NULL};
    const char* pngExts[]  = {".png", NULL};

    if(hasExtension(fname, jpgExts)) {
        return getJPEGSize(fname, width, height, throwException);
    } else if(hasExtension(fname, pngExts)) {
        return getPNGSize(fname, width, height, throwException);
    }
    return 0;
}

int
getJPEGSize(const char* fname, int& width, int& height, bool throwException)
{
    FILE* file = fopen(fname, "rb");
    if(file == NULL) {
        if(throwException) {
            std::stringstream err;
            err << "Could not open file " << fname << " for reading";
            throw sfmf::IOError(err.str());
        }
        return 0;
    }

    int result = scanhead(file, &width, &height);
    fclose(file);

    return result;
}

int
getPNGSize(const char* fname, int& width, int& height, bool throwException)
{
    // Ref: http://stackoverflow.com/questions/5354459/c-how-to-get-the-image-size-of-a-png-file-in-directory
    std::ifstream in(fname);
    if(!in.good()) {
        if(throwException) {
            std::stringstream err;
            err << "Could not open file " << fname << " for reading";
            throw sfmf::IOError(err.str());
        }
        return 0;
    }

    unsigned int widthTmp, heightTmp;

    in.seekg(16);
    in.read((char*)&widthTmp, 4);
    in.read((char*)&heightTmp, 4);

    widthTmp = ntohl(widthTmp);
    heightTmp = ntohl(heightTmp);
    width = widthTmp;
    height = heightTmp;

    return 1;
}

void
colormapValues(const std::vector<double>& values,
               std::vector<Eigen::Vector3f>& colors,
               std::string* mapping)
{
    std::vector<double> valuesSorted = values;
    std::sort(valuesSorted.begin(), valuesSorted.end());
    double quants[4];
    quants[0] = valuesSorted[0];
    quants[1] = valuesSorted[values.size() * 0.33];
    quants[2] = valuesSorted[values.size() * 0.66];
    quants[3] = valuesSorted[values.size() - 1];

    Eigen::Vector3f quantColors[4] = {
        Eigen::Vector3f(0, 0, 1),
        Eigen::Vector3f(0, 1, 0),
        Eigen::Vector3f(1, 1, 0),
        Eigen::Vector3f(1, 0, 0)
    };

    if(mapping != NULL) {
        std::stringstream mappingS;
        mappingS << "Color mapping\n";
        const char* fmt = "%6.2f -> Color = [%.1f, %.1f, %.1f]\n";
        for (int i = 0; i < 4; i++) {
            char buf[1000];
            sprintf(buf, fmt, quants[i], quantColors[i][0], quantColors[i][1], quantColors[i][2]);
            mappingS << buf;
        }
        *mapping = mappingS.str();
    }

    int idx = 0;
    for(std::vector<Eigen::Vector3f>::iterator it = colors.begin(), itEnd = colors.end(); it != itEnd; it++, idx++) {
        double value = values[idx];

        int i = 0;
        for(; value >= quants[i]; ++i);
        i = std::min(i, 3);
        assert(i >= 0 && i < 4);

        float alpha = float(value - quants[i - 1]) / float(quants[i] - quants[i - 1]);

        assert(alpha <= 1.0 && alpha >= 0.0);

        *it = quantColors[i - 1] * alpha + (1.0 - alpha) * quantColors[i];
    }
}

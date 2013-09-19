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

#ifndef __SFMF_IO_HPP__
#define __SFMF_IO_HPP__

// This class can read a standard text file or a GZip compressed
// file. It checks the first few bytes of the file to determine
// if the file was compressed.
class CompressedFileReader
{
public:
    CompressedFileReader(const char* filename, bool throwException = true);

    bool good() {
        return _in.good();
    }

    template<typename T>
    friend
    CompressedFileReader&
    operator>>(CompressedFileReader& r, T& v);

private:
    boost::iostreams::filtering_istream _in;
};

template<typename T>
CompressedFileReader&
operator>>(CompressedFileReader& r, T& v)
{
    r._in >> v;
    return r;
}

#endif // __SFMF_IO_HPP__
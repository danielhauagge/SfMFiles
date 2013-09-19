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

#ifndef __UTILS_HPP__
#define __UTILS_HPP__

/// Determines size of image (only reads first few bytes of file)
/// @returns 0 on failure and non zero on success
int getImageSize(const char* fname, int& width, int& height, bool throwException = true);

/// Determines size of image stored as JPEG file (only reads first few bytes of file)
/// @returns 0 on failure and non zero on success
int getJPEGSize(const char* fname, int& width, int& height, bool throwException = true);

/// Determines size of image stored as PNG file (only reads first few bytes of file)
/// @returns 0 on failure and non zero on success
int getPNGSize(const char* fname, int& width, int& height, bool throwException = true);

/// Computes a color mapping from a vector of real numbers
/// @returns text describing the color mapping (to be inserted
/// as a comment into the ply file)
void colormapValues(const std::vector<double>& values,
                    std::vector<Eigen::Vector3f>& colors,
                    std::string* mapping = NULL);

#endif // __UTILS_HPP__
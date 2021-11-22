/*
 *  Copyright (c) 2009-2011, NVIDIA Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of NVIDIA Corporation nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "io/ImagePfmIO.hpp"
#include "gui/Image.hpp"
#include "io/Stream.hpp"

using namespace FW;

//------------------------------------------------------------------------

Image* FW::importPfmImage(InputStream& stream)
{
    // Read header fields.

    String fields[4];
    for (int fieldIdx = 0; fieldIdx < FW_ARRAY_SIZE(fields);)
    {
        // Read character.

        char c = stream.readU8();
        if (hasError())
            break;

        // Whitespace => advance to the next field.

        if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
        {
            fieldIdx++;
            continue;
        }

        // Field is too long => error.

        if (fields[fieldIdx].getLength() >= 1024)
        {
            setError("Corrupt PFM header!");
            break;
        }

        // Append character to the field.

        fields[fieldIdx] += c;
    }

    // Parse field 0: format identifier.

    int numChannels = -1;
    if (fields[0] == "PF") // RGB
        numChannels = 3;
    else if (fields[0] == "Pf") // grayscale
        numChannels = 1;
    else
        setError("Invalid PFM format identifier!");

    // Parse field 1: width.

    int width = -1;
    const char* p1 = fields[1].getPtr();
    if (!parseInt(p1, width) || *p1 || width < 1)
        setError("Invalid PFM width!");

    // Parse field 2: height.

    int height = -1;
    const char* p2 = fields[2].getPtr();
    if (!parseInt(p2, height) || *p2 || height < 1)
        setError("Invalid PFM height!");

    // Parse field 3: scale factor.

    F32 scale = 0.0f; // positive: big endian, negative: little endian
    const char* p3 = fields[3].getPtr();
    if (!parseFloat(p3, scale) || *p3 || !(scale > 0.0f || scale < 0.0f))
        setError("Invalid PFM scale factor!");

    // Check for errors.

    if (hasError())
        return NULL;

    // Create image.
    // TODO: Use a different image format for grayscale?

    Image* image = new Image(Vec2i(width, height), ImageFormat::RGB_Vec3f);
    U32* pixels = (U32*)image->getMutablePtr();

    // Read raster data in bottom-up order.

    for (int y = height - 1; y >= 0; y--)
        stream.readFully(&pixels[y * width * numChannels], width * numChannels * sizeof(U32));

    if (hasError())
    {
        delete image;
        return NULL;
    }

    // Grayscale => expand to RGB.

    if (numChannels == 1)
        for (int i = width * height - 1; i >= 0; i--)
            pixels[i * 3 + 0] = pixels[i * 3 + 1] = pixels[i * 3 + 2] = pixels[i];

    // Wrong endianess => reverse bytes.

    if ((scale > 0.0f) != (*(const U32*)"\x01\x02\x03\x04" == 0x01020304))
    {
        int n = width * height * 3;
        for (int i = 0; i < n; i++)
        {
            U32 v = pixels[i];
            pixels[i] = (v >> 24) | ((v >> 8) & 0x0000FF00u) | ((v << 8) & 0x00FF0000u) | (v << 24);
        }
    }
    return image;
}

//------------------------------------------------------------------------

void FW::exportPfmImage(OutputStream& stream, const Image* image)
{
    // Convert image to RGB_Vec3f.

    FW_ASSERT(image);
    Vec2i size = image->getSize();
    bool empty = (size.min() <= 0);
    const Image* source = image;
    Image* converted = NULL;

    if (empty ||
        image->getFormat().getID() != ImageFormat::RGB_Vec3f ||
        image->getStride() != size.x * (int)sizeof(Vec3f))
    {
        size = Vec2i(max(size.x, 1), max(size.y, 1));
        converted = new Image(size, ImageFormat::RGB_Vec3f);
        if (empty)
            converted->clear();
        else
            *converted = *image;
        source = converted;
    }

    // Write header.

    bool bigEndian = (*(const U32*)"\x01\x02\x03\x04" == 0x01020304);
    String header = sprintf("PF\n%d %d\n%g\n", size.x, size.y, (bigEndian) ? 1.0f : -1.0f);
    stream.write(header.getPtr(), header.getLength());

    // Write raster data in bottom-up order.

    const Vec3f* pixels = (const Vec3f*)source->getPtr();
    for (int y = size.y - 1; y >= 0; y--)
        stream.write(&pixels[y * size.x], size.x * sizeof(Vec3f));

    // Clean up.

    delete converted;
}

//------------------------------------------------------------------------

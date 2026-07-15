#include "lodepng.h"
#include <algorithm>
#include <cassert>
#include <vector>
#include <cstdint>

namespace img_parser {

class image_parser
{
public:
    int getWidth() const
    {
        return width_;
    }

    int getHeight() const
    {
        return height_;
    }

    image_parser(const std::string &imageFilePath)
    {
        unsigned error = lodepng::decode(imageArray_, width_, height_, imageFilePath, LCT_RGB);
        assert(!error && "Error: Unable to load the PNG file.");
    }

    // This function will now return an RGB triplet
    std::vector<uint8_t> getPixelValue(int y, int x)
    {
        int index = 3 * (y * width_ + x);
        return {imageArray_[index], imageArray_[index + 1], imageArray_[index + 2]};
    }

    // This function will now update an RGB triplet
    void updatePixel(int y, int x, const std::vector<uint8_t> &rgb)
    {
        if (y < 0 || y >= static_cast<int>(height_) || x < 0 || x >= static_cast<int>(width_))
            return; // Out of bounds check
        int index = 3 * (y * width_ + x);
        imageArray_[index] = rgb[0];
        imageArray_[index + 1] = rgb[1];
        imageArray_[index + 2] = rgb[2];
    }

    void drawLine(int x0, int y0, int x1, int y1)
    {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;

        while (true)
        {
            setPixelAreaToRed(y0, x0); // Coloring the pixel and its surrounding area

            if (x0 == x1 && y0 == y1)
                break;
            e2 = err;
            if (e2 > -dx)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dy)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    // Function to set the pixel to red
    void setPixelAreaToRed(int y, int x, int radius = 1)
    {
        for (int i = -radius; i <= radius; i++)
        {
            for (int j = -radius; j <= radius; j++)
            {
                std::vector<uint8_t> redPixel = {255, 0, 0}; // Assuming RGBA format
                updatePixel(y + i, x + j, redPixel);
            }
        }
    }

    bool isNeighborBlack(int y, int x, uint8_t threshold) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dy == 0 && dx == 0) continue; //skip self
                int ny = std::clamp(y + dy, 0, static_cast<int>(height_) - 1);
                int nx = std::clamp(x + dx, 0, static_cast<int>(width_) - 1);
                std::vector<uint8_t> neighborPixel = getPixelValue(ny, nx);
                if (neighborPixel[0] < threshold && neighborPixel[1] < threshold &&
                    neighborPixel[2] < threshold) {
                    return true;
                }
            }
        }
        return false;
    }

    bool isPixelWhite(int y, int x, uint8_t threshold) {
        std::vector<uint8_t> pixelValue = getPixelValue(y, x);
        return (pixelValue[0] > threshold && pixelValue[1] > threshold && pixelValue[2] > threshold);
    }


    void dialateMazeBruteForce(int level, uint8_t threshold = 128){
        for(int l = 0; l < level; l++) {
            std::vector<std::pair<int, int>> whitePixels;
            for (int y = 0; y < static_cast<int>(height_); y++) {
                for (int x = 0; x < static_cast<int>(width_); x++) {
                    if(isPixelWhite(y, x, threshold))
                    {
                        if(isNeighborBlack(y, x, threshold))
                            whitePixels.push_back(std::make_pair(x, y));
                    }
                }
            }
            // Now set the surrounding pixels of all identified white pixels to white
            for (const auto &p: whitePixels) {
                int x = p.first;
                int y = p.second;
                std::vector<uint8_t> blackPixel(3, threshold-1);
                updatePixel(y, x, blackPixel);
            }
        }
    }

    void dilateMaze(int level, uint8_t threshold = 128)
    {
        // take 2d temp array to track distance from black pixels
        std::vector<std::vector<uint8_t>> tempImage(height_, std::vector<uint8_t>(width_, 255));
        for (int y = 0; y < static_cast<int>(height_); y++)
        {
            for (int x = 0; x < static_cast<int>(width_); x++)
            {
                if(!isPixelWhite(y, x, threshold))
                {
                    tempImage[y][x] = 0; // black pixel
                }
            }
        }
        // forward pass to calculate distance
        for (int y = 0; y < static_cast<int>(height_); y++) {
            for (int x = 0; x < static_cast<int>(width_); x++) {
                if (tempImage[y][x] != 0) {
                    uint8_t minDist = 255;
                    if (x > 0)
                        minDist = std::min(minDist, static_cast<uint8_t>(tempImage[y][x - 1] + 1));
                    if (y > 0)
                        minDist = std::min(minDist, static_cast<uint8_t>(tempImage[y - 1][x] + 1));
                    tempImage[y][x] = std::min(tempImage[y][x], minDist);
                }
            }
        }

        // backward pass to calculate distance
        for (int y = static_cast<int>(height_) - 1; y >= 0; y--) {
            for (int x = static_cast<int>(width_) - 1; x >= 0; x--) {
                if (tempImage[y][x] != 0) {
                    uint8_t minDist = 255;
                    if (x + 1 < static_cast<int>(width_))
                        minDist = std::min(minDist, static_cast<uint8_t>(tempImage[y][x + 1] + 1));
                    if (y + 1 < static_cast<int>(height_))
                        minDist = std::min(minDist, static_cast<uint8_t>(tempImage[y + 1][x] + 1));
                    tempImage[y][x] = std::min(tempImage[y][x], minDist);
                }
            }
        }
        // update original image based on distance
        for (int y = 0; y < static_cast<int>(height_); y++) {
            for (int x = 0; x < static_cast<int>(width_); x++) {
                if (tempImage[y][x] <= level && tempImage[y][x] > 0) {
                    std::vector<uint8_t> blackPixel(3, threshold - 1);
                    updatePixel(y, x, blackPixel);
                }
            }
        }

    }

    void writeImage(const std::string &outputFileName)
    {
        unsigned error = lodepng::encode(outputFileName, imageArray_, width_, height_, LCT_RGB);
        assert(!error && "Error: Unable to save the PNG file.");
    }

private:
    std::vector<uint8_t> imageArray_;
    unsigned width_, height_;
};


}
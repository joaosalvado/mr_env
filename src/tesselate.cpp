//
// Created by ohmy on 2022-04-27.
//

#include "../include/mrenv/Tesselation.h"

int main(int argc, char** argv)
{
    // params
    std::string map_file {argv[1]};
    int length = std::atoi(argv[2]);
    int width = std::atoi(argv[3]);

    // Initialize
    mrenv::Tesselation tessel{};
    tessel.setFootprint(length, width);
    tessel.inputScenario(map_file);

    // Compute Rectangles
    tessel.coverRectangles();

    // Plot the computed Rectangles
    tessel.plotBestCover();

    // Solution, list of recantgles
    auto rects  = tessel.getRectangles();

    return 1;
}
#include "../include/mrenv/Tesselation.h"
#include <iostream>




int main()
{
        //Parameters
        auto seed_point = Point2d(360, 230);
        int length = 500;
        int width = 500;

        std::string map_file = "basement.png";
        //std::string maps_path = "../../maps/";

        mrenv::Tesselation tessel;
        tessel.setFootprint(length, width);
        tessel.inputScenario(map_file);


        tessel.coverRectangles();
        tessel.plotBestCover();

        auto rects  = tessel.getRectangles();

      int a = 0;
}


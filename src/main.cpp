#include "../include/mrenv/Tesselation.h"
#include <iostream>



int main()
{
        //Parameters
        auto seed_point = Point2d(360, 230);
        int length = 1;
        int width = 1;




        std::string map_file = "map-partial-1.yaml";
        //std::string maps_path = "../../maps/";

        mrenv::Tesselation tessel;
        tessel.inputScenario(map_file, length, width);


        tessel.coverRectangles();
        tessel.plotBestCover();


        
}


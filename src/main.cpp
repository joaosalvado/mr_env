#include "../include/mrenv/Tesselation.h"
#include <iostream>




int main()
{
        //Parameters
        auto seed_point = Point2d(360, 230);
        int length = 1000;
        int width = 1000;

        std::string map_file = "random.png";
        std::string maps_path = "/home/ohmy/js_ws/github_joao/mrenv/maps/";

        mrenv::Tesselation tessel;
        tessel.setFootprint(length, width);
        tessel.addPathToScenarios(maps_path);
        tessel.inputScenario(map_file);
        //tessel.addCountours();
        //tessel.computePolyhedra(seed_point.x, seed_point.y);
        //tessel.maxRectangle(360, 230);

        tessel.coverRectangles();
        tessel.plotBestCover();

        
}


#ifndef TESSELATION
#define TESSELATION
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <Eigen/Core>
#include "iris/iris.h"
#include <list>
#include <queue>

using namespace cv;

// auto cmp = [this](Rectangle rectA, Rectangle rectb){
//     return area(rectA) < area(rectB);
// };
namespace mrenv
{
    class Tesselation
    {
        typedef std::vector<Point> Polygon;
        typedef struct rectangle_
        {
            Point2d left_bottom_corner;
            Point2d right_upper_corner;
        } Rectangle;
        
        struct compareRectangle
        {
            bool operator()(
                std::shared_ptr<Rectangle> rectA,
                std::shared_ptr<Rectangle> rectB)
            {
                return area(*rectA) < area(*rectB);
            }
        };

        typedef struct cover
        {
            std::list<std::shared_ptr<Rectangle>> rectangles;
            double area;
            int num_pol;

        } cover;

    public:
        Tesselation(){};
        virtual ~Tesselation(){};
        void inputScenario(std::string file_name)
        {
            //READ IMAGE
            std::string image_path = samples::findFile(file_name);

            color_img = imread(image_path, IMREAD_COLOR);
            cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);

            if (color_img.empty())
            {
                std::cout << "Could not read the image: " << image_path << std::endl;
                return;
            }

            best_cover_ = nullptr;
        }
        void addPathToScenarios(std::string maps_folder_path)
        {
            samples::addSamplesDataSearchPath(maps_folder_path);
        }

        void coverRectangles();

        void plotBestCover();
        void setFootprint(int Lenght, int width);

        void doubleImage();

    private:
        int length_px, width_px;
        std::list<Polygon> polygons;
        Mat color_img, gray_img;
        std::vector<std::vector<Point>> contours;
        std::shared_ptr<cover> best_cover_;

        void addCountours();
        void computePolyhedra(double seed_x, double seed_y);
        bool isColliding(const Point2d &left_bottom_corner, const Point2d &right_upper_corner);
        void createRectangle(
            const Point2d &center,
            int pos_x, int neg_x,
            int pos_y, int neg_y,
            std::shared_ptr<mrenv::Tesselation::Rectangle> &rect);
        std::shared_ptr<Rectangle> maxRectangle(int seed_x, int seed_y);
        double computeCoverArea(const cover &cov);
        static double area(const Rectangle &rect);

        //Images
        void addConvexPolygon(Mat img, const Point *points, int n_pts);
        void addRectangle(Mat img, Point corner1, Point corner2);
        void addFilledCircle(Mat img, Point center);
        void addLine(Mat img, Point start, Point end);
        void fillPolygonBlack(Mat img, const Point *points, int n_pts);
        void fillRectangle(Mat img, Point corner1, Point corner2);
        void whiteRectangle(Mat img, Point corner1, Point corner2);
        void blueRectangle(Mat img, Point corner1, Point corner2);
    };

} // namespace mrenv

#endif
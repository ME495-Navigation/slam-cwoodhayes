#include "turtlelib/svg.hpp"

namespace turtlelib
{
    std::string SvgSpec::to_svg_elem()
    {
    }

    Svg::Svg(std::pair<double, double> xbound, std::pair<double, double> ybound)
    {
    }

    std::string Svg::draw(turtlelib::Point2D p)
    {
    }

    std::string Svg::draw(turtlelib::Vector2D v)
    {
    }

    std::string Svg::draw(turtlelib::Transform2D f)
    {
    }

    void Svg::write_file(std::filesystem::path svg_path, SvgSpec spec)
    {
    }
}

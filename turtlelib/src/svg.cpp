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
        return "";
    }

    std::string Svg::draw(turtlelib::Vector2D v)
    {
        return "";
    }

    std::string Svg::draw(turtlelib::Transform2D f)
    {
        return "";
    }

    void Svg::write_file(std::filesystem::path svg_path, SvgSpec spec)
    {
    }
}

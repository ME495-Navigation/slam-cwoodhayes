#include "turtlelib/svg.hpp"

#include <format>
#include <fstream>
#include <sstream>

namespace turtlelib
{
    std::string Svg::draw(turtlelib::Point2D p, std::string color)
    {
        auto id = std::to_string(elements_.size());
        elements_.insert({id, std::make_unique<DrawablePoint>(id, p, color)});
        return id;
    }

    std::string Svg::draw(turtlelib::Vector2D v, std::string color)
    {
        auto id = std::to_string(elements_.size());
        elements_.insert({id, std::make_unique<DrawableVector>(id, v, color)});
        return id;
    }

    std::string Svg::draw(turtlelib::AbsVector av, std::string color)
    {
        auto id = std::to_string(elements_.size());
        elements_.insert({id, std::make_unique<DrawableAbsVector>(id, av, color)});
        return id;
    }

    std::string Svg::draw(turtlelib::Transform2D f, std::string name)
    {
        auto id = std::to_string(elements_.size());
        elements_.insert({id, std::make_unique<DrawableFrame>(id, f, name)});
        return id;
    }

    std::string Svg::to_string(turtlelib::SvgSpec spec) const
    {
        auto vec_def = R"'''(
<defs>
  <marker style="overflow:visible" id="Arrow1Sstart" refX="0.0" refY="0.0" orient="auto">
       <path transform="scale(0.2) translate(6,0)" style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt" d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "/>
    </marker>
</defs>
        )'''";

        std::stringstream ss;
        ss << spec.to_svg_elem() << "\n";
        ss << vec_def << "\n";

        // write all elements
        for (const auto& [id, element] : elements_) {
            auto svg_element = element->draw(spec);
            ss << svg_element << "\n";
        }
        ss << "</svg>\n";

        return ss.str();
    }

    void Svg::write_file(std::filesystem::path svg_path, SvgSpec spec)
    {
        // open the file for writing
        if (!svg_path.parent_path().empty()) {
            std::filesystem::create_directories(svg_path.parent_path());
        }
        std::ofstream ofile(svg_path);

        if (!ofile) {
            throw std::runtime_error("Could not open file: " + svg_path.string());
        }

        ofile << to_string(spec);
    }
}
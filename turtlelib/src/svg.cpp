#include "turtlelib/svg.hpp"

#include <format>
#include <fstream>

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

    std::string Svg::draw(turtlelib::Transform2D f, std::string name)
    {
        auto id = std::to_string(elements_.size());
        elements_.insert({id, std::make_unique<DrawableFrame>(id, f, name)});
        return id;
    }

    void Svg::write_file(std::filesystem::path svg_path, SvgSpec spec)
    {
        auto vec_def = R"'''(
<defs>
  <marker style="overflow:visible" id="Arrow1Sstart" refX="0.0" refY="0.0" orient="auto">
       <path transform="scale(0.2) translate(6,0)" style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt" d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "/>
    </marker>
</defs>
        )'''";
        // open the file for writing
        if (!svg_path.parent_path().empty()) {
            std::filesystem::create_directories(svg_path.parent_path());
        }
        std::ofstream ofile(svg_path);

        if (!ofile) {
            throw std::runtime_error("Could not open file: " + svg_path.string());
        }

        // write svg header and vector definition
        ofile << spec.to_svg_elem() << "\n";
        ofile << vec_def << "\n";

        // write all elements
        for (const auto& [id, element] : elements_) {
            auto svg_element = element->draw(spec);
            ofile << svg_element << "\n";
        }
        ofile << "</svg>\n";
    }
}
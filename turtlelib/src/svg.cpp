#include "turtlelib/svg.hpp"

#include <format>
#include <fstream>

namespace turtlelib
{
    std::string SvgSpec::to_svg_elem()
    {
        return std::format(
            "<svg width=\"{:.6f}in\" height=\"{:.6f}in\" viewBox=\"{:.6f} {:.6f} {:.6f} {:.6f}\" xmlns=\"http://www.w3.org/2000/svg\">",
            page_width_in,
            page_height_in,
            box_origin_px.first,
            box_origin_px.second,
            box_bottom_right_px.first,
            box_bottom_right_px.second
        );
    }

    std::string Svg::draw(turtlelib::Point2D p)
    {
        auto id = std::to_string(elements_.size());
        elements_.insert({id, std::make_unique<DrawablePoint>(p)});
        return id;
    }

    std::string Svg::draw(turtlelib::Vector2D v)
    {
        auto id = std::to_string(elements_.size());
        elements_.insert({id, std::make_unique<DrawableVector>(v)});
        return id;
    }

    std::string Svg::draw(turtlelib::Transform2D f)
    {
        auto id = std::to_string(elements_.size());
        elements_.insert({id, std::make_unique<DrawableFrame>(f)});
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
        // string templates for each element type

        auto template_point = R"'''(
<circle id={} cx="{:.1f}" cy="{:.1f}" r="3" stroke="purple" fill="purple" stroke-width="1"/>
        )'''";
        auto template_vector = R"'''(
<line id={} x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="purple" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
        )'''";
        auto template_frame = R"'''(
<g id={}>
    <line x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <line x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <text x="{:.6f}" y="{:.6f}">{a}</text>
</g>
        )'''";


        // open the file for writing
        if (!svg_path.parent_path().empty()) {
            std::filesystem::create_directories(svg_path.parent_path());
        }
        std::ofstream ofile(svg_path);

        if (!ofile) {
            throw std::runtime_error("Could not open file: " + svg_path.string());
        }

        // Write SVG header
        ofile << spec.to_svg_elem() << "\n";

        // ##################### Begin_Citation [2] ####################
        for (const auto& [id, element] : elements_) {
            auto svg_element = element->draw();
            ofile << svg_element << "\n";
        }
        // ##################### End_Citation [2] ####################

        // Close SVG
        ofile << "</svg>\n";
    }


    // TODO refactor into separate file
    std::string DrawablePoint::draw() const {
        return "point";
    }

    std::string DrawableVector::draw() const {
        return "vec";
    }

    std::string DrawableFrame::draw() const {
        return "frame";
    }
}
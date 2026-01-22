/// \file
/// \brief implementations for svg_elements.hpp

#include "turtlelib/svg_elements.hpp"

namespace turtlelib {
    std::pair<double, double> SvgSpec::user_point_to_svg_point(Point2D p) {
        // TODO fill in
        return {0.0, 0.0};
    }

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

    std::string DrawablePoint::draw(SvgSpec spec) const {

        constexpr auto fmt = R"'''(
<circle id={} cx="{:.1f}" cy="{:.1f}" r="3" stroke="purple" fill="purple" stroke-width="1"/>
        )'''";
        auto spt = spec.user_point_to_svg_point(point_);

        return std::format(fmt,
            0, spt.first, spt.second
        );
    }

    std::string DrawableVector::draw(SvgSpec spec) const {
        // vector tails are always at the origin
        constexpr auto fmt = R"'''(
<line id={} x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="purple" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
        )'''";
        auto vpt = spec.user_point_to_svg_point(Point2D() + vector_);
        return std::format(fmt,
            0, vpt.first, spec.uf_origin_px.first, vpt.second, spec.uf_origin_px.second
        );
    }

    std::string DrawableFrame::draw(SvgSpec spec) const {
        constexpr auto fmt = R"'''(
<g id={}>
    <line x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <line x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <text x="{:.6f}" y="{:.6f}">{a}</text>
</g>
        )'''";

        // hardcoding the size of these in svg canvas-space for now.
        auto axis_len = 96.0;

        // y axis point
        Transform2D({96.0});
        
        auto origin = frame_.translation();
        return std::string();
    }
}
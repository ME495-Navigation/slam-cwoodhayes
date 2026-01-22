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
<circle id={} cx="{:.1f}" cy="{:.1f}" r="3" stroke="{}" fill="{}" stroke-width="1"/>
        )'''";
        auto p_px = spec.user_point_to_svg_point(point_);

        return std::format(fmt,
            id_, p_px.first, p_px.second, color_, color_
        );
    }

    std::string DrawableVector::draw(SvgSpec spec) const {
        // vector tails are always at the origin
        constexpr auto fmt = R"'''(
<line id={} x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="{}" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
        )'''";
        auto v_px = spec.user_point_to_svg_point(Point2D() + vector_);
        return std::format(fmt,
            id_, v_px.first, spec.uf_origin_px.first, v_px.second, spec.uf_origin_px.second, color_
        );
    }

    std::string DrawableFrame::draw(SvgSpec spec) const {
        constexpr auto fmt = R"'''(
<g id={}>
    <line x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="{}" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <line x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="{}" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <text x="{:.6f}" y="{:.6f}">{}</text>
</g>
        )'''";

        // hardcoding the size of these in svg canvas-space for now.
        auto axis_len = 96.0;
        std::pair<double, double> name_offset = {5.0, 5.0};

        // derive the head locations of the axes.
        auto y_head = tf_(Point2D(0, axis_len));
        auto x_head = tf_(Point2D(axis_len, 0));

        auto yh_px = spec.user_point_to_svg_point(y_head);
        auto xh_px = spec.user_point_to_svg_point(x_head);
        auto origin_px = spec.uf_origin_px;

        return std::format(fmt,
            id_,
            origin_px.first, xh_px.first, origin_px.second, xh_px.second, color_,
            origin_px.first, yh_px.first, origin_px.second, yh_px.second, y_color_,
            origin_px.first + name_offset.first, origin_px.second - name_offset.second, name_
        );
        
        return std::string();
    }
}
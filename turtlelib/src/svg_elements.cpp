/// \file
/// \brief implementations for svg_elements.hpp

#include "turtlelib/svg_elements.hpp"

namespace turtlelib {
    std::pair<double, double> SvgSpec::user_point_to_svg_point(Point2D p) {
        // hardcoding this per matt's instructions in the example svg below:
        /*
     The viewBox sets the conversion from pixels to inches and is
     the frame in which you draw.

     The viewBox frame, in this case has:
       96 pixels per inch (a line 96 pixels long is one inch long).
       The origin is in the upper-left corner of the page.
       The x pixel value increases to the right.
       The y pixel value increases down.
       (This is a left-handed coordinate system).

     When drawing, we will conceive of the image as follows:
       1 unit in turtlelib (e.g., meters) corresponds to 1 inch in the svg.

       The "fixed frame" is implicitly the midpoint of the page (in viewBox coordinates),
       with x axis positive to the right and y axis positive up.

       Therefore coordinate frame resulting from the identity transform
       is drawn at the center of the page with x-axis right and y axis up.
        */
        constexpr double pixels_per_unit = 96.0;  // 96 pixels per inch
        
        // Convert from user frame (right-handed, origin at center) to SVG frame (left-handed, origin at top-left)
        // SVG x increases right (same as user frame)
        // SVG y increases down (opposite of user frame, so we subtract)
        double svg_x = uf_origin_px.first + p.x * pixels_per_unit;
        double svg_y = uf_origin_px.second - p.y * pixels_per_unit;
        
        return {svg_x, svg_y};
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
<circle id="{}" cx="{:.1f}" cy="{:.1f}" r="3" stroke="{}" fill="{}" stroke-width="1"/>
        )'''";
        auto p_px = spec.user_point_to_svg_point(point_);

        return std::format(fmt,
            id_, p_px.first, p_px.second, color_, color_
        );
    }

    std::string DrawableVector::draw(SvgSpec spec) const {
        // vector tails are always at the origin
        constexpr auto fmt = R"'''(
<line id="{}" x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="{}" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
        )'''";
        auto v_px = spec.user_point_to_svg_point(Point2D() + vector_);
        return std::format(fmt,
            id_, v_px.first, spec.uf_origin_px.first, v_px.second, spec.uf_origin_px.second, color_
        );
    }

    std::string DrawableFrame::draw(SvgSpec spec) const {
        constexpr auto fmt = R"'''(
<g id="{}">
    <line x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="{}" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <line x1="{:.6f}" x2="{:.6f}" y1="{:.6f}" y2="{:.6f}" stroke="{}" stroke-width="5" marker-start="url(#Arrow1Sstart)"/> /&gt;
    <text x="{:.6f}" y="{:.6f}">{{{}}}</text>
</g>
        )'''";

        // hardcoding the size of these in user-frame for now.
        auto axis_len = 1.0;
        std::pair<double, double> name_offset = {5.0, 5.0};

        // derive the head locations of the axes.
        auto y_head = tf_(Point2D(0, axis_len));
        auto x_head = tf_(Point2D(axis_len, 0));
        auto origin = tf_(Point2D());

        auto yh_px = spec.user_point_to_svg_point(y_head);
        auto xh_px = spec.user_point_to_svg_point(x_head);
        auto origin_px = spec.user_point_to_svg_point(origin);

        return std::format(fmt,
            id_,
            xh_px.first, origin_px.first, xh_px.second, origin_px.second, color_,
            yh_px.first, origin_px.first, yh_px.second, origin_px.second, y_color_,
            origin_px.first + name_offset.first, origin_px.second - name_offset.second, name_
        );
        
        return std::string();
    }
}
#ifndef TURTLELIB_SVG_ELEMENTS_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_ELEMENTS_INCLUDE_GUARD_HPP
/// \file
/// \brief SVG drawing element classes.

#include <string>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    /// @brief Spec representation for an SVG document
    struct SvgSpec {
        /// @brief page width in inches
        double page_width_in = 8.5;
        /// @brief page height in inches
        double page_height_in = 11.0;

        /// @brief (x, y) position of the viewBox origin
        std::pair<double, double> box_origin_px = {0, 0};
        /// @brief (x, y) position of the viewBox bottom right corner.
        std::pair<double, double> box_bottom_right_px = {816.0, 1056.0};
        /// @brief (x, y) position of the User Frame origin in svg space.
        std::pair<double, double> uf_origin_px = {408.0, 528.0};

        /// @brief Default constructor using default values
        SvgSpec() {};

        /// @brief Convert this spec to an <svg> element.
        /// @return string containing <svg width=...> 
        std::string to_svg_elem();

        /// @brief Convert a point in User Frame to a point in SVG pixel space.
        /// @param p point in user frame
        /// @return x, y point in pixel space. using pair to make it explicit we're not in user frame anymore.
        std::pair<double, double> user_point_to_svg_point(Point2D p);
    };

    /// @brief Abstract base class for drawable elements in SVG
    /// These elements speak in SVG-frame (left handed), not User Frame.
    class DrawingElement {
    public:
        virtual ~DrawingElement() = default;
        
        /// @brief Render the element to an SVG string
        /// @return SVG representation of this element
        virtual std::string draw(SvgSpec spec) const = 0;
        
    protected:
        /// @brief Constructor for drawable element with an ID and color
        /// @param id unique identifier for this element in the SVG
        /// @param color stroke color for this element
        explicit DrawingElement(std::string id, std::string color) : id_(id), color_(color) {}
        
        /// @brief The unique ID of this element
        std::string id_;
        /// @brief The stroke color for this element
        std::string color_;
    };

    /// @brief Drawable point element
    class DrawablePoint : public DrawingElement {
    public:
        /// @brief Construct a drawable point
        /// @param id unique identifier in the SVG
        /// @param p the point to draw
        /// @param color stroke color (default: purple)
        DrawablePoint(std::string id, Point2D p, std::string color = "purple") : DrawingElement(id, color), point_(p) {}
        
        /// @brief Draw the point as an SVG element
        /// @return SVG string representation
        std::string draw(SvgSpec spec) const override;
        
    private:
        Point2D point_;
    };

    /// @brief Drawable vector element
    class DrawableVector : public DrawingElement {
    public:
        /// @brief Construct a drawable vector
        /// @param id unique identifier in the SVG
        /// @param v the vector to draw
        /// @param color stroke color (default: purple)
        DrawableVector(std::string id, Vector2D v, std::string color = "purple") : DrawingElement(id, color), vector_(v) {}
        
        /// @brief Draw the vector as an SVG element
        /// @return SVG string representation
        std::string draw(SvgSpec spec) const override;
        
    private:
        Vector2D vector_;
    };

    /// @brief Drawable coordinate frame element
    class DrawableFrame : public DrawingElement {
    public:
        /// @brief Construct the drawable frame
        /// @param id unique identifier in the SVG
        /// @param f transform to the frame
        /// @param name name of the frame
        /// @param x_color stroke color for x-axis (default: red)
        /// @param y_color stroke color for y-axis (default: green)
        DrawableFrame(std::string id, Transform2D f, std::string name, std::string x_color = "red", std::string y_color = "green") 
            : DrawingElement(id, x_color), frame_(f), name_(name), y_color_(y_color) {}
        
        /// @brief Draw the coordinate frame as an SVG element
        /// @return SVG string representation
        std::string draw(SvgSpec spec) const override;
        
    private:
        Transform2D frame_;
        std::string name_;
        std::string y_color_;
    };
};

#endif // TURTLELIB_SVG_ELEMENTS_INCLUDE_GUARD_HPP

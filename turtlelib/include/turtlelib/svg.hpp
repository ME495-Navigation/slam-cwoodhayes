#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief SVG visualization class.

#include <string>
#include <filesystem>
#include <unordered_map>
#include <memory>
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

        /// @brief Default constructor using default values
        SvgSpec() {};

        /// @brief Convert this spec to an <svg> element.
        /// @return string containing <svg width=...> 
        std::string to_svg_elem();
    };

    /// @brief Abstract base class for drawable elements in SVG
    class DrawingElement {
    public:
        virtual ~DrawingElement() = default;
        
        /// @brief Render the element to an SVG string
        /// @return SVG representation of this element
        virtual std::string draw() const = 0;
    };

    /// @brief Drawable point element
    class DrawablePoint : public DrawingElement {
    public:
        DrawablePoint(Point2D p) : point_(p) {}
        
        /// @brief Draw the point as an SVG element
        /// @return SVG string representation
        std::string draw() const override;
        
    private:
        Point2D point_;
    };

    /// @brief Drawable vector element
    class DrawableVector : public DrawingElement {
    public:
        DrawableVector(Vector2D v) : vector_(v) {}
        
        /// @brief Draw the vector as an SVG element
        /// @return SVG string representation
        std::string draw() const override;
        
    private:
        Vector2D vector_;
    };

    /// @brief Drawable coordinate frame element
    class DrawableFrame : public DrawingElement {
    public:
        DrawableFrame(Transform2D f) : frame_(f) {}
        
        /// @brief Draw the coordinate frame as an SVG element
        /// @return SVG string representation
        std::string draw() const override;
        
    private:
        Transform2D frame_;
    };

    /// @brief create and modify simple .svg drawings
    /// User specifies points, vectors, and coordinate frames in their own coordinate frame
    /// and only needs to define the SVG output file's frame when writing to the file.
    class Svg {
    public:
        /// @brief Construct an SVG canvas to represent 2D space
        /// @param xbound X limits of the SVG canvas in your 2D coordinate frame (User Frame) 
        /// (can be arbitrary i.e. not corresponding to pixels)
        /// @param ybound Y limits of the SVG canvas in your 2D coordinate frame (User Frame).
        Svg(std::pair<double, double> xbound, std::pair<double, double> ybound) : xbound_(xbound), ybound_(ybound) {}

        /// @brief Draw a point on the canvas
        /// @param p point specified in User Frame
        /// @return string ID of the point in the svg
        std::string draw(turtlelib::Point2D p);

        /// @brief Draw a vector on the canvas
        /// @param v vector specified in User Frame
        /// @return string ID of the vector in the svg
        std::string draw(turtlelib::Vector2D v);

        /// @brief Draw a coordinate frame on the canvas
        /// @param f frame location & orientation relative to User Frame
        /// @return ID of the coordinate frame in the svg
        std::string draw(turtlelib::Transform2D f);

        /// @brief Write the canvas to an SVG file.
        /// @param svg_path path the output file
        /// @param spec spec for the svg file to write
        void write_file(std::filesystem::path svg_path, SvgSpec spec = {});

    private:
        std::unordered_map<std::string, std::unique_ptr<DrawingElement>> elements_;

        std::pair<double, double> xbound_;
        std::pair<double, double> ybound_;
    };
};

#endif // TURTLELIB_SVG_INCLUDE_GUARD_HPP

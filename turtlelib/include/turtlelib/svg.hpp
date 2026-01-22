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
#include "turtlelib/svg_elements.hpp"

namespace turtlelib
{

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
        /// @param color stroke and fill color (default: purple)
        /// @return string ID of the point in the svg
        std::string draw(turtlelib::Point2D p, std::string color = "purple");

        /// @brief Draw a vector on the canvas
        /// @param v vector specified in User Frame
        /// @param color stroke color (default: purple)
        /// @return string ID of the vector in the svg
        std::string draw(turtlelib::Vector2D v, std::string color = "purple");

        /// @brief Draw an absolute vector on the canvas (with specified tail and head)
        /// @param av absolute vector with both tail and head points specified in User Frame
        /// @param color stroke color (default: purple)
        /// @return string ID of the vector in the svg
        std::string draw(turtlelib::AbsVector av, std::string color = "purple");

        /// @brief Draw a coordinate frame on the canvas
        /// @param f frame location & orientation relative to User Frame
        /// @param name name for the coordinate frame
        /// @return ID of the coordinate frame in the svg
        std::string draw(turtlelib::Transform2D f, std::string name);

        /// @brief Write the canvas to an SVG file.
        /// @param svg_path path the output file
        /// @param spec spec for the svg file to write
        void write_file(std::filesystem::path svg_path, turtlelib::SvgSpec spec = {});

        /// @brief Generate the SVG content as a string
        /// @param spec spec for the svg file to write
        /// @return string containing the complete SVG markup
        std::string to_string(turtlelib::SvgSpec spec = {}) const;

    private:
        std::unordered_map<std::string, std::unique_ptr<DrawingElement>> elements_;

        std::pair<double, double> xbound_;
        std::pair<double, double> ybound_;
    };
};

#endif // TURTLELIB_SVG_INCLUDE_GUARD_HPP

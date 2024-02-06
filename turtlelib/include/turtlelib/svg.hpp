#ifndef SVG_HPP
#define SVG_HPP

#include <vector>
#include <iostream>
#include <fstream>

/// \file
/// \brief SVG image generator

namespace turtlelib
    {
    /// \brief SVG image generator class
    class SVG {
    private:
        /// \brief Width of the image
        int width;
        /// \brief Height of the image
        int height;
        /// \brief Elements of the svg file
        std::vector<std::string> elements;

    public:
        /// \brief Creates the image
        /// \param w Width of the image
        /// \param h Height of the image
        SVG(int w, int h);

        /// \brief Saves the image as a named file
        /// \param filename Desired file name
        /// \param extra image file that is modified
        // is this object in  valid state after calling this function?
        void save_file(const std::string& filename, const std::string& extra);

        /// \brief Adds a line in the image
        /// \param x1 x coord of the beginning of the line
        /// \param x2 x coord of the end of the line
        /// \param y1 y coord of the beginning of the line
        /// \param y2 y coord of the end of the line
        /// \param color Color of the line
        void draw_line(int x1, int y1, int x2, int y2, const std::string& color);

        /// \brief Adds a rectangle to the image
        /// \param x Rectangle's x coordinate
        /// \param y Rectangle's y coordinate
        /// \param w Rectangle's width
        /// \param h Rectangle's height
        void draw_rectangle(int x, int y, int w, int h);

        /// \brief Adds a citcle to the image
        /// \param x Circle center's x coordinate
        /// \param y Circle center's y coordinate
        /// \param rad Circle's radius
        /// \param color Color of the circle
        void draw_circle(int x, int y, int rad, const std::string& color);

        /// \brief Adds text to the image
        /// \param x Text box's x coordinate
        /// \param y Text box's y coordinate
        /// \param text Text added to the image
        void add_text(int x, int y, const std::string& text);
        
        /// \brief Returns the image width
        /// \return Width of the image
        int get_width(); // const
        
        /// \brief Returns the image height
        /// \return Height of the image
        int get_height(); // const
    };
}

#endif // SVG_HPP

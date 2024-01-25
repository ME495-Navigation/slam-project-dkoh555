#include "turtlelib/svg.hpp"

SVG::SVG(int w, int h) : width(w), height(h) {
    elements.push_back("<svg width=\"" + std::to_string(w) + "\" height=\"" + std::to_string(h) + "\">");
}

void SVG::save_file(const std::string& filename, const std::string& extra) {
    elements.push_back(extra);
    elements.push_back("</svg>");
    
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const std::string& element : elements) {
            file << element << "\n";
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}

void SVG::draw_line(int x1, int y1, int x2, int y2, const std::string& color) {
    elements.push_back("<line x1=\"" + std::to_string(x1) + "\" y1=\"" + std::to_string(y1) +
                       "\" x2=\"" + std::to_string(x2) + "\" y2=\"" + std::to_string(y2) + "\" stroke=\"" + color + "\" stroke-width=\"2\" marker-start=\"url(#Arrow1Sstart)\"/>");
}

void SVG::draw_rectangle(int x, int y, int w, int h) {
    elements.push_back("<rect x=\"" + std::to_string(x) + "\" y=\"" + std::to_string(y) +
                       "\" width=\"" + std::to_string(w) + "\" height=\"" + std::to_string(h) + "\"/>");
}

void SVG::draw_circle(int x, int y, int rad, const std::string& color) {
    elements.push_back("<circle cx=\"" + std::to_string(x) + "\" cy=\"" + std::to_string(y) +
                       "\" r=\"" + std::to_string(rad) + "\" stroke=\"" + color + "\" fill=\"" + color + "\" stroke-width=\"1\"/>");
}

void SVG::add_text(int x, int y, const std::string& text) {
    elements.push_back("<text x=\"" + std::to_string(x) + "\" y=\"" + std::to_string(y) + "\">" + text + "</text>");
}

int SVG::get_width(){
    return width;
}

int SVG::get_height(){
    return height;
}

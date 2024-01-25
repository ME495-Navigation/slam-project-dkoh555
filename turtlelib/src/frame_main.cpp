#include <iostream>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <cmath>
#include "turtlelib/svg.hpp"

using turtlelib::SVG;
using turtlelib::Transform2D;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;

// Declare helper function
Vector2D normalizeVector(const Vector2D & v);

int main()
{
    SVG svg(816, 1056);

    std::string svgCode = R"(
    <defs>
    <marker
        style="overflow:visible"
        id="Arrow1Sstart"
        refX="0.0"
        refY="0.0"
        orient="auto">
        <path
            transform="scale(0.2) translate(6,0)\"
            style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt"
            d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "
            />
        </marker>
    </defs>

    <g>
    
    </g>
    )";

    Transform2D Tab, Tbc;

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> Tbc;

    Transform2D Tba{Tab.inv().translation(), Tab.inv().rotation()};
    Transform2D Tcb{Tbc.inv().translation(), Tbc.inv().rotation()};
    Transform2D Tac{(Tab*Tbc).translation(), (Tab*Tbc).rotation()};
    Transform2D Tca{Tac.inv().translation(), Tac.inv().rotation()};

    std::cout << "T_{a, b}: " << Tab << "\n";
    std::cout << "T_{b, a}: " << Tba << "\n";
    std::cout << "T_{b, c}: " << Tbc << "\n";
    std::cout << "T_{c, b}: " << Tcb << "\n";
    std::cout << "T_{a, c}: " << Tac << "\n";
    std::cout << "T_{c, a}: " << Tca << std::endl;

    Point2D origin_a{408, 728};
    Vector2D x_axis{1,0}, y_axis{0,1};
    double axis_length = 100.0;
    double unit_length = axis_length;

    // Frame a
    svg.draw_line(origin_a.x + axis_length, origin_a.y, origin_a.x, origin_a.y, "red");
    svg.draw_line(origin_a.x, origin_a.y - axis_length, origin_a.x, origin_a.y, "green");
    svg.add_text(origin_a.x, origin_a.y, "{a}");

    // Frame b
    Point2D origin_b{origin_a.x + unit_length * Tab.translation().x, origin_a.y - unit_length * Tab.translation().y}; 

    svg.draw_line(origin_b.x + axis_length * Tab(x_axis).x, origin_b.y - axis_length * Tab(x_axis).y, origin_b.x, origin_b.y, "red");
    svg.draw_line(origin_b.x + axis_length * Tab(y_axis).x, origin_b.y - axis_length * Tab(y_axis).y, origin_b.x, origin_b.y, "green");
    svg.add_text(origin_b.x, origin_b.y, "{b}");

    // Frame c
    Point2D origin_c{origin_a.x + unit_length * Tac.translation().x, origin_a.y - unit_length * Tac.translation().y}; 

    svg.draw_line(origin_c.x + axis_length * Tac(x_axis).x, origin_c.y - axis_length * Tac(x_axis).y, origin_c.x, origin_c.y, "red");
    svg.draw_line(origin_c.x + axis_length * Tac(y_axis).x, origin_c.y - axis_length * Tac(y_axis).y, origin_c.x, origin_c.y, "green");
    svg.add_text(origin_c.x, origin_c.y, "{c}");

    Point2D pa{}, pb{}, pc{};
    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> pa;
    pb = Tba(pa);
    pc = Tca(pa);

    std::cout << "p_a " << pa << "\n";
    std::cout << "p_b " << pb << "\n";
    std::cout << "p_c " << pc << std::endl;

    // Point a
    svg.draw_circle(origin_a.x + unit_length * pa.x, origin_a.y - unit_length * pa.y, 5, "purple");
    // Point b
    svg.draw_circle(origin_a.x + unit_length * Tab(pb).x, origin_a.y - unit_length * Tab(pb).y, 5, "brown");
    // Point c
    svg.draw_circle(origin_a.x + unit_length * Tac(pc).x, origin_a.y - unit_length * Tac(pc).y, 5, "orange");

    Vector2D vb{}, vb_hat{}, va{}, vc{};
    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> vb;
    vb_hat = normalizeVector(vb);
    va = Tab(vb);
    vc = Tcb(vb);
    
    std::cout << "v^_b " << vb_hat << "\n";
    std::cout << "v_a " << va << "\n";
    std::cout << "v_b " << vb << "\n";
    std::cout << "v_c " << vc << std::endl;
    
    // Draw v^_b
    svg.draw_line(origin_b.x + axis_length * Tab(vb_hat).x, origin_b.y - axis_length * Tab(vb_hat).y, origin_b.x, origin_b.y, "brown");
    // Draw v_a
    svg.draw_line(origin_a.x + axis_length * va.x, origin_a.y - axis_length * va.y, origin_a.x, origin_a.y, "purple");
    // Draw v_c
    svg.draw_line(origin_c.x + axis_length * Tac(vc).x, origin_c.y - axis_length * Tac(vc).y, origin_c.x, origin_c.y, "orange");

    Twist2D Va, Vb, Vc;

    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> Vb;
    Va = Tab(Vb);
    Vc = Tcb(Vb);
    std::cout << "V_a " << Va << "\n";
    std::cout << "V_b " << Vb << "\n";
    std::cout << "V_c " << Vc << std::endl;

    // Save the provided SVG code to the file
    svg.save_file("../tmp/frames.svg", svgCode);
    return 0;
}

// Helper function
Vector2D normalizeVector(const Vector2D & v) {
    Vector2D v_hat{};
    double v_norm = sqrt(v.x * v.x + v.y * v.y);

    if(v_norm == 0.0)
    {
        std::cout << "Invalid vector" << std::endl;
    }
    else
    {
        v_hat.x = v.x / v_norm;
        v_hat.y = v.y / v_norm;
    }

    return v_hat;
}
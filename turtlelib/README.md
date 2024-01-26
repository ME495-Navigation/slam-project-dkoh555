# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.


# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?

   **Different Implementations:**

   1. Implementation no.1:
   ```
    // Free helper function (not tied to a class or struct)
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
    }

    return v_hat;
   ```

   2. Implementation no.2:
   ```
    // Member helper function
    void normalize() {
        double magnitude = std::sqrt(x^2 + y^2);

        if (magnitude == 0.0)
        {
            std::cout << "Invalid vector" << std::endl;
        }
        else
        {
            x = x / magnitude;
            y = y / magnitude;
        }
    }
   ```

   3. Implementation no.3:
   ```
    // Operator overloading for normalization
    Vector2D operator/(double divisor) const {
        // Check for division by zero to avoid NaN
        if (divisor == 0.0) {
            return *this;
        }
        return {x / divisor, y / divisor};
    }

    // Calculate the magnitude
    double magnitude() const {
        return std::sqrt(x^2 + y^2);
    }

    // Final function for vector normalization
    Vector2D normalize() const {
        double new_mag = magnitude();
        return *this / new_mag; // Using the above overloaded division operator
    }
   ```
   **Pros & Cons:**

   1. Pros: The C++ Core Guidelines bring the Seperation of Concerns, which recommends seperating concerns to improve the code's maintainability; Cons: C.9 suggests not using free functions if they do not directly access any private members, which breaks encapsulation.

   2. Pros: From the C++ Core Guidelines, to achieve encapsulation C.9 strongly suggests using member functions for operations involving objects instantiated by a class; Cons: C.4 suggests to minimize the of functions that modify the object.

   3. Pros: According to the C++ Core Guidelines, F.47 says to maintain consistency with standard library types; Cons: F.15 advises against overloading operators in ways that users might not find intuitive, where this method of normalization may be unituitive to some people.

   **Which method I would implement:**

   I chose implementation no.1 (free helper function) as this came the most intuitively to me, and I prefer to have a clear return object or value as this makes reading and maintaining my code much more straightforward.


2. What is the difference between a class and a struct in C++?
    
    In a class, any and all members are private by default unless explicity stated. Meaning that said members are only accessible from within the class unless specified such as using public, protected, or private keywords.

    In contrast, the members in a struct are public by default. Therefore, functions and other means are able to access said members unless stated otherwise.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

    According to the C++ Core Guidelines, C.8 states that a class is preferred over a struct when it contains public members. Additionally, C.2 also recommends using a class if the object maintains an invariant - meaning that its upholding a rule that must be maintained throught its lifetime.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

    In the C++ Core Guidelines, C.46 states: By default, declare single-argument constructors explicit.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

    The Transform2D::inv() method is declared as const because it does not modify any member variables of the Transform2D object it belongs to.
    
    However, the Transform2D::operator*=() performs an assignment operation that alters the state of the Transform2D object, therefore it is not immutable, as they normally are supposed to be.
#ifndef RANDOM_NUMBER_GENERATOR_HELPERS_HPP
#define RANDOM_NUMBER_GENERATOR_HELPERS_HPP

#include <random>
#include "rclcpp/rclcpp.hpp"

namespace num_generator
{
    /// @brief declare a parameter without a default value. If the value is not set externally,
    /// an exception will be thrown when trying to get_param for this parameter.
    /// @tparam T - type of the parameter
    /// @param name - name of the parameter
    /// @param node - node for which the parameter is declared
    /// @param desc - (optional) the parameter description
    /// @throwD
    ///   rclcpp::exceptions::ParameterAlreadyDeclaredException - if the parameter has already been declared
    ///   rclcpp::exceptions::UninitializedStaticallyTypedParameterException - if the parameter was not set when the node is run
    std::mt19937 & get_random()
    {
        // static variables inside a function are created once and persist for the remainder of the program
        static std::random_device rd{}; 
        static std::mt19937 mt{rd()};
        // we return a reference to the pseudo-random number genrator object. This is always the
        // same object every time get_random is called
        return mt;
    }
}

#endif
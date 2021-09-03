/****************************************************************************
 Module
    vector_operators.hpp
 Description
    This is a set of vector overload operator and vector function definition 
    intended to improve ease of use of standard vector data types for basic 
    linear algebra and vector operation math.

*****************************************************************************/
#ifndef VECTOR_OPERATORS_HPP_
#define VECTOR_OPERATORS_HPP_
/*----------------------------- Include Files ------------------------------*/
#include <vector>

/*-------------------------------- Defines ---------------------------------*/

/*--------------------------------- Types ----------------------------------*/

/*------------------------------- Variables --------------------------------*/

/*---------------------- Public Function Prototypes ------------------------*/

//template <class T>
std::vector<double> operator+(const std::vector<double>& a, const std::vector<double>& b);
std::vector<double> operator-(const std::vector<double>& a, const std::vector<double>& b);
std::vector<double> operator*(const std::vector<double>& a, const std::vector<double>& b);
std::vector<double> operator/(const std::vector<double>& a, const std::vector<double>& b);
std::vector<double> operator==(const std::vector<double>& a, const std::vector<double>& b);
std::vector<double> operator^(const std::vector<double>& a, double b);
std::vector<double> operator&(const std::vector<double>& a, double b);
std::vector<double> operator|(const std::vector<double>& a, double b);
double vector_sum(std::vector<double> a);
std::vector<double> normalize(std::vector<double> a);
/*------------------------------ End of file -------------------------------*/
#endif /* VECTOR_OPERATORS_HPP_ */

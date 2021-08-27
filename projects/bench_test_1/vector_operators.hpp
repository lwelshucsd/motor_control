/****************************************************************************
 Module
    module_name.h
 Description
    description of module
 Notes
    additional notes

 History
 When             Who    What/Why
 --------------   ---    --------
 DD MMMM YYYY     XXX    changes
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
double vsum(std::vector<double> a);
double norm(std::vector<double> a);
/*------------------------------ End of file -------------------------------*/
#endif /* VECTOR_OPERATORS_HPP_ */

/****************************************************************************
 Module
	vector_operators.cpp
 Description
	This is a set of vector overload operator and vector function definition
	intended to improve ease of use of standard vector data types for basic
	linear algebra and vector operation math.

*****************************************************************************/

/*----------------------------- Include Files ------------------------------*/
#include "vector_operators.hpp"
#include <iostream>
#include <algorithm>
#include <functional>
#include <numeric>
#include <cassert>

/*--------------------------- External Variables ---------------------------*/
/*----------------------------- Module Defines -----------------------------*/
/*------------------------------ Module Types ------------------------------*/
/*---------------------------- Module Variables ----------------------------*/

/*--------------------- Module Function Prototypes -------------------------*/
/*------------------------------ Module Code -------------------------------*/

// Vector Operators
//template <class T>

std::vector<double> operator+(const std::vector<double>& a, const std::vector<double>& b) {

	/// Summary: Performs elementwise vector addition on two double vectors a & b

	assert(a.size() == b.size());

	std::vector<double> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
		std::back_inserter(result), std::plus<double>());
	return result;
}

std::vector<double> operator-(const std::vector<double>& a, const std::vector<double>& b) {

	/// Summary: Performs elementwise vector subtraction on two double vectors a & b

	assert(a.size() == b.size());

	std::vector<double> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
		std::back_inserter(result), std::minus<double>());
	return result;
}

std::vector<double> operator*(const std::vector<double>& a, const std::vector<double>& b) {

	/// Summary: Performs elementwise vector multiplication on two double vectors a & b

	assert(a.size() == b.size());

	std::vector<double> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
		std::back_inserter(result), std::multiplies<double>());
	return result;
}

std::vector<double> operator/(const std::vector<double>& a, const std::vector<double>& b) {

	/// Summary: Performs elementwise vector division on two double vectors a & b

	assert(a.size() == b.size());

	std::vector<double> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
		std::back_inserter(result), std::divides<double>());
	return result;
}

std::vector<double> operator==(const std::vector<double>& a, const std::vector<double>& b) {

	/// Summary: evaluates elementwise equivalence for two double vectors a & b
	/// Returns: vector of 1 or 0

	assert(a.size() == b.size());

	std::vector<double> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
		std::back_inserter(result), std::equal_to<double>());
	return result;
}

std::vector<double> operator^(const std::vector<double>& a, double b) {

	/// Summary: Raises all elements in vector a to exponential power b

	std::vector<double> result;
	for (size_t i = 0; i < a.size(); i++) {
		result.push_back(pow(a[i], b));
	}
	return result;
}

std::vector<double> operator&(const std::vector<double>& a, double b) {

	/// Summary: adds scalar b to each element in vector a

	std::vector<double> result;
	result.resize(a.size());
	transform(a.begin(), a.end(), result.begin(), [&a, &b](auto& c) {return c + b; });
	return result;
}

std::vector<double> operator|(const std::vector<double>& a, double b) {

	/// Summary: Multiplies each element in vector a by scalar b

	std::vector<double> result;
	result.resize(a.size());
	transform(a.begin(), a.end(), result.begin(), [&a, &b](auto& c) {return c * b; });
	return result;
}

double vector_sum(std::vector<double> a) {

	/// Summary: evaluates the sum total of double vector a

	double sum = 0;
	for (size_t i = 0; i < a.size(); i++) { sum += a[i]; }
	//return(std::accumulate(a.begin(), a.end(), 0));
	return sum;
}

std::vector<double> normalize(std::vector<double> a) {

	/// Summary: Normalizes vector a

	double b = sqrt(vector_sum(a ^ 2));		// Take Norm of Vector
	b = 1 / b;							// Take reciprocal of norm
	std::vector<double> res = a | b;	// Multiply vector by reciprocal of norm
	return res;
}

///// Vector overload operator definition template
//std::vector<double> operator+(const std::vector<double>& a, const std::vector<double>& b) {
//	assert(a.size() == b.size());
//
//	std::vector<double> result;
//	result.reserve(a.size());
//
//	std::transform(a.begin(), a.end(), b.begin(),
//		std::back_inserter(result), std::plus<double>());
//	return result;
//}



/*----------------------------- Test Harness -------------------------------*/

/*------------------------------- Footnotes --------------------------------*/
/*------------------------------ End of file -------------------------------*/

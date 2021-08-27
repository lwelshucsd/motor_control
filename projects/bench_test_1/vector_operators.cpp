/****************************************************************************
 Module
	module_name.c
 Description
	description of module
 Notes
	additional notes

 History
 When             Who    What/Why
 --------------   ---    --------
 DD MMMM YYYY     XXX    changes
*****************************************************************************/

/*----------------------------- Include Files ------------------------------*/
#include "vector_operators.hpp"
#include <math.h>
#include <Windows.h>
#include <valarray>
#include <algorithm>
#include <functional>
#include <cassert>
#include <numeric>
#include "general_functions.hpp"

/*--------------------------- External Variables ---------------------------*/
/*----------------------------- Module Defines -----------------------------*/
/*------------------------------ Module Types ------------------------------*/
/*---------------------------- Module Variables ----------------------------*/

/*--------------------- Module Function Prototypes -------------------------*/
/*------------------------------ Module Code -------------------------------*/

// Vector Operators
//template <class T>

std::vector<double> operator+(const std::vector<double>& a, const std::vector<double>& b) {
	assert(a.size() == b.size());

	std::vector<double> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
		std::back_inserter(result), std::plus<double>());
	return result;
}

std::vector<double> operator-(const std::vector<double>& a, const std::vector<double>& b) {
	assert(a.size() == b.size());

	std::vector<double> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
		std::back_inserter(result), std::minus<double>());
	return result;
}

std::vector<double> operator*(const std::vector<double>& a, const std::vector<double>& b) {
	assert(a.size() == b.size());

	std::vector<double> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
		std::back_inserter(result), std::multiplies<double>());
	return result;
}

std::vector<double> operator/(const std::vector<double>& a, const std::vector<double>& b) {
	assert(a.size() == b.size());

	std::vector<double> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
		std::back_inserter(result), std::divides<double>());
	return result;
}

std::vector<double> operator==(const std::vector<double>& a, const std::vector<double>& b) {
	assert(a.size() == b.size());

	std::vector<double> result;
	result.reserve(a.size());

	std::transform(a.begin(), a.end(), b.begin(),
		std::back_inserter(result), std::equal_to<double>());
	return result;
}

std::vector<double> operator^(const std::vector<double>& a, double b) {
	std::vector<double> result;
	for (size_t i = 0; i < a.size(); i++) {
		result.push_back(pow(a[i], b));
	}
	return result;
}

std::vector<double> operator&(const std::vector<double>& a, double b) {
	std::vector<double> result;
	result.resize(a.size());
	transform(a.begin(), a.end(), result.begin(), [&a, &b](auto& c) {return c + b; });
	return result;
}

std::vector<double> operator|(const std::vector<double>& a, double b) {
	std::vector<double> result;
	result.resize(a.size());
	transform(a.begin(), a.end(), result.begin(), [&a, &b](auto& c) {return c * b; });
	return result;
}

double vsum(std::vector<double> a) {
	double sum = 0;
	for (size_t i = 0; i < a.size(); i++) { sum += a[i]; }
	//return(std::accumulate(a.begin(), a.end(), 0));
	return sum;
}

double norm(std::vector<double> a) {
	return(sqrt(vsum(a ^ 2)));
}

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

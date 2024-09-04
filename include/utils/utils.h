#pragma once

inline double normal_pdf(const double x, const double mu, const double sigma) {
	const auto exponent = -(x - mu) * (x - mu) / (2.0 * sigma * sigma);
	return (1.0 / (sigma * sqrt(2.0 * M_PI))) * pow(M_E, exponent);
}
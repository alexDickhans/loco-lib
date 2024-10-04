#pragma once

/**
 * @brief Uses a an inverse quintic to approximate the normal function
 *
 * @param x Number of standard deviations from mean
 * @return Approximation of the normal pdf
 */
inline float cheap_norm_pdf(const float x) {
    // Approximation of the standard normal PDF
    // Coefficients for the rational approximation
    const float a = 0.3989422804014337;
    const float e = 0.59422804014337;

    // Compute the approximate normal PDF using a rational polynomial
    const float pdfApprox = a / (1.0 + e * x * x * x * x);

    return pdfApprox;
}

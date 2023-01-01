#include "trilaterate.h"

vec3 trilaterate(vec3 c1, vec3 c2, vec3 c3, double r1, double r2, double r3, bool pos) {
    // first, transform space so that c1 is at (0,0,0)
    vec3 c2trf = c2 - c1;
    vec3 c3trf = c3 - c1;

    // calculate w_hat and b_hat
    vec3 w_hat = c2trf / c2trf.length();
    vec3 b = c3trf - proj(c3trf,w_hat);
    vec3 b_hat = b / b.length();
    vec3 k_hat = cross(w_hat, b_hat);

    // calculate the variables we need to solve for the point using the wikipedia solution
    double u = c2trf.length();
    double Vx = proj(c3trf, w_hat).length();
    double Vy = proj(c3trf, b_hat).length();

    // solve for the x, y, and z components of the center point
    double x = ((r1 * r1) - (r2 * r2) + (u * u)) / (2 * u);
    double y = ((r1 * r1) - (r3 * r3) + (Vx * Vx) + (Vy * Vy) - (2 * Vx * x)) / (2 * Vy);
    double z = sqrt(r1*r1 - x*x - y*y) * (pos ? 1: -1);
    vec3 centertrf = {x, y, z}; // This is a coord in the transformed space

    // transform the solution back into original space
    vec3 center = c1 + centertrf[0] * w_hat + centertrf[1] * b_hat + centertrf[2] * k_hat;

    return center;
}
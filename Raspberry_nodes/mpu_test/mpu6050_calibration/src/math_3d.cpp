/**
 * @author  Dwindra Sulistyoutomo
 */

#include "math_3d.h"

// Quaternion
Quaternion::Quaternion(){
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
}
Quaternion::Quaternion(float w_in, float x_in, float y_in, float z_in){
    w = w_in;
    x = x_in;
    y = y_in;
    z = z_in;
}

float Quaternion::GetMagnitude(){
    return sqrt(w*w + x*x + y*y + z*z);
}

void Quaternion::Normalize(){
    float m = Quaternion::GetMagnitude();
    w /= m;
    x /= m;
    y /= m;
    z /= m;
}

Quaternion Quaternion::GetNormalizedQuaternion(){
    Quaternion q(w,x,y,z);
    q.Normalize();
    return q;
}

Quaternion Quaternion::GetConjugate(){
    return Quaternion(w, -x, -y, -z);
}

/**
 * Hamilton Product
 * q' =   (w1w2 - x1x2 - y1y2 - z1z2) 
 *      + (w1x2 + x1w2 + y1z2 - z1y2)i
 *      + (w1y2 - x1z2 + y1w2 + z1x2)j
 *      + (w1z2 + x1y2 - y1x2 + z1w2)k
 */
Quaternion Quaternion::GetProduct(Quaternion q){
    return Quaternion(
        w*q.w - x*q.x - y*q.y - z*q.z,
        w*q.x + x*q.w + y*q.z - z*q.y,
        w*q.y - x*q.z + y*q.w + z*q.x,
        w*q.z + x*q.y - y*q.x + z*q.w
    );
}

// Vector
Vector::Vector(){
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
}
Vector::Vector(float x_in, float y_in, float z_in){
    x = x_in;
    y = y_in;
    z = z_in;
}

float Vector::GetMagnitude(){
    return sqrt(x*x + y*y + z*z);
}

void Vector::Normalize(){
    float m = Vector::GetMagnitude();
    x /= m;
    y /= m;
    z /= m;
}
Vector Vector::GetNormalizedVector(){
    Vector v(x,y,z);
    v.Normalize();
    return v;
}

/**
 * p' = q * p * conj(q)
 */
void Vector::Rotate(Quaternion q){
    // Create Quaternion from current vector
    Quaternion p(0,x,y,z);

    // Do the first Hamilton Product
    p = q.GetProduct(p);
    // Do the second Hamilton Product
    p = p.GetProduct(q.GetConjugate());

    // Update the current vector from the output quaternion
    x = p.x;
    y = p.y;
    z = p.z;
}

Vector Vector::GetRotatedVector(Quaternion q){
    Vector v(x,y,z);
    v.Rotate(q);
    return v;
}

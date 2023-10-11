/**
 * @author  Dwindra Sulistyoutomo
 */

#ifndef _MATH_3D_H
#define _MATH_3D_H

#include <math.h>

class Quaternion {
    public:
        float w;
        float x;
        float y;
        float z;

        // Constructor
        Quaternion();
        Quaternion(float w_in, float x_in, float y_in, float z_in);

        /**
         * Return the magnitude of the quaternion
         */
        float GetMagnitude();

        /**
         * Normalize the current quaternion
         */
        void Normalize();

        /**
         * Return the normalized quaternion from the current quaternion
         */
        Quaternion GetNormalizedQuaternion();
        
        /**
         * Return the conjugate of the quaternion
         * Conjugate has same magnitude, but has opposite sign for imaginary parts
         */
        Quaternion GetConjugate();

        /**
         * Return the Hamilton Product of current Quaternion q1 with given Quaternion q2
         * q1 = w1 + x1i + y1j + z1k
         * q2 = w2 + x2i + y2j + z2k
         * 
         * Hamilton product: q' = q1 * q2
         * q' =   (w1w2 - x1x2 - y1y2 - z1z2) 
         *      + (w1x2 + x1w2 + y1z2 - z1y2)i
         *      + (w1y2 - x1z2 + y1w2 + z1x2)j
         *      + (w1z2 + x1y2 - y1x2 + z1w2)k
         * 
         * @param q {Quaternion} Quaternion to be multiplied into current Quaternion
         */
        Quaternion GetProduct(Quaternion q);
};

class Vector {
    public:
        float x;
        float y;
        float z;

        // Constructor
        Vector();
        Vector(float x_in, float y_in, float z_in);

        /**
         * Return the magnitude of the vector
         */
        float GetMagnitude();

        /**
         * Normalize the current vector
         */
        void Normalize();
        
        /**
         * Return the normalized vector from the current vector
         */
        Vector GetNormalizedVector();

        /**
         * Rotate the vector with given rotation quaternion
         * p' = q * p * conj(q)
         * 
         * In order to calculate, vector need to be converted into quaternion
         * p = xi + yj + kj  -> p = 0 + xi + yj + zk
         * 
         * @param q {Quaternion} rotation quaternion 
         */
        void Rotate(Quaternion q);

        /**
         * Return the rotated vector from thic vector with given quaternion
         * 
         * @param q {Quaternion} rotation quaternion
         */
        Vector GetRotatedVector(Quaternion q);
};

#endif
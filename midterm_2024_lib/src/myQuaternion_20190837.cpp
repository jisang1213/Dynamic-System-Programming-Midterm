#include "../include/myQuaternion.hpp"
#include <limits> // For std::numeric_limits

#include <stdio.h>

//////////////////// TO DO  ////////////////////
// TODO: refer "myQuaternion.hpp" and complete below functions.
Quaternion::Quaternion() {
}

Quaternion::Quaternion(double w, double x, double y, double z) : Vec4(w,x,y,z) {
}

double Quaternion::w() {
    return get_elem(0,0);
}

double Quaternion::x() {
    return get_elem(1,0);
}

double Quaternion::y() {
    return get_elem(2,0);
}

double Quaternion::z() {
    return get_elem(3,0);
}

void Quaternion::inversed(){
    double w=this->w(), x=this->x(), y=this->y(), z=this->z();
    double squarednorm = this->squarednorm();
    w = w/squarednorm;
    x = -1*x/squarednorm;
    y = -1*y/squarednorm;
    z = -1*z/squarednorm;
    set_elem(0,0,w);
    set_elem(1,0,x);
    set_elem(2,0,y);
    set_elem(3,0,z);
}

void Quaternion::normalized(){
    double w=this->w(), x=this->x(), y=this->y(), z=this->z();
    double norm = sqrt(w*w + x*x + y*y + z*z);
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
    set_elem(0,0,w);
    set_elem(1,0,x);
    set_elem(2,0,y);
    set_elem(3,0,z);
}

void Quaternion::display()
{
    double w=this->w(), x=this->x(), y=this->y(), z=this->z();
    printf("Quaternion = {w : %f, x : %f, y : %f, z: %f}\n", w,x,y,z);
}

Quaternion getQuaternionBetweenTimeStep(Vec3 w, double delta_t){
    double norm = sqrt(w.squarednorm());
    double angle = norm*delta_t;
    double x,y,z;
    x = w.get_elem(0,0)/norm;
    y = w.get_elem(1,0)/norm;
    z = w.get_elem(2,0)/norm;
    return Quaternion(std::cos(angle/2), std::sin(angle/2)*x, std::sin(angle/2)*y, std::sin(angle/2)*z);
}

Quaternion quatmulquat(Quaternion q1, Quaternion q2){
    double a1,b1,c1,d1;
    double a2,b2,c2,d2;
    double w,x,y,z;

    a1 = q1.w();
    b1 = q1.x();
    c1 = q1.y();
    d1 = q1.z();

    a2 = q2.w();
    b2 = q2.x();
    c2 = q2.y();
    d2 = q2.z();

    w = a1*a2 - b1*b2 - c1*c2 - d1*d2;
    x = a1*b2 + b1*a2 + c1*d2 - d1*c2;
    y = a1*c2 - b1*d2 + c1*a2 +d1*b2;
    z = a1*d2 + b1*c2 - c1*b2 + d1*a2;

    return Quaternion(w,x,y,z);
}

Vec3 vecRotatedByQuat(Vec3 v, Quaternion q){
    Quaternion vec(0, v.get_elem(0,0), v.get_elem(1,0), v.get_elem(2,0));
    Quaternion q_inv = q;
    q_inv.inversed();
    vec = quatmulquat(q, quatmulquat(vec, q_inv));
    Vec3 rotated_vec(vec.get_elem(1,0), vec.get_elem(2,0), vec.get_elem(3,0));
    return rotated_vec;
}

Quaternion mat2quat(const Mat33& mat){
    double a11,a12,a13, a21,a22,a23, a31,a32,a33;

    a11 = mat.get_elem(0,0);
    a12 = mat.get_elem(0,1);
    a13 = mat.get_elem(0,2);

    a21 = mat.get_elem(1,0);
    a22 = mat.get_elem(1,1);
    a23 = mat.get_elem(1,2);

    a31 = mat.get_elem(2,0);
    a32 = mat.get_elem(2,1);
    a33 = mat.get_elem(2,2);

    double trace = a11 + a22 + a33;
    double t;

    double w,x,y,z;

    if (trace > 0) {
        t = 1 + trace;
        w = t;
        x = a32 - a23;
        y = a13 - a31;
        z = a21 - a12;
    } else if (a11 > a22 && a11 > a33) {
        t = 1 + a11 - a22 - a33;
        w = a32 - a23;
        x = t;
        y = a21 + a12;
        z = a31 + a13;
    } else if (a22 > a33) {
        t = 1 - a11 + a22 - a33;
        w = a13 - a31;
        x = a21 + a12;
        y = t;
        z = a32 + a23;
    } else {
        t = 1 - a11 - a22 + a33;
        w = a21 - a12;
        x = a31 + a13;
        y = a32 + a23;
        z = t;
    }

    Quaternion result(w,x,y,z);
    result.normalized();
    return result;
}

Mat33 quat2mat(Quaternion& q) {
    Mat33 mat;
    double w,x,y,z;

    q.normalized();

    w = q.w();
    x = q.x();
    y = q.y();
    z = q.z();

    mat.set_elem(0,0, 1 - 2*y*y - 2*z*z);
    mat.set_elem(0,1, 2*x*y - 2*w*z);
    mat.set_elem(0,2, 2*x*z + 2*w*y);

    mat.set_elem(1,0, 2*x*y + 2*w*z);
    mat.set_elem(1,1, 1 - 2*x*x -2*z*z);
    mat.set_elem(1,2, 2*y*z - 2*w*x);

    mat.set_elem(2,0, 2*x*z - 2*w*y);
    mat.set_elem(2,1, 2*y*z + 2*w*x);
    mat.set_elem(2,2, 1 - 2*x*x -2*y*y);

    return mat;
}
//////////////////// TO DO end ////////////////////

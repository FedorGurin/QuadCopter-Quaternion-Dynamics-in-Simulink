#pragma once

#ifndef MATH_FUNC_H
#define MATH_FUNC_H

#include "constants.h"

/*! \defgroup math Математика
 *  \brief Векторы, кватернионы и вспомогательные функции
 *  (аналог библиотеки QuaternionLib.slx)
 *  \ingroup kernel
*/
namespace math_func {

//! Трехмерный вектор \ingroup math
class TVector3
{
public:
    TVector3(double x_ = 0.0, double y_ = 0.0, double z_ = 0.0)
    {
        x = x_;
        y = y_;
        z = z_;
    }
    double x;
    double y;
    double z;

    double& operator[](int i)       {return (i == 0) ? x : ((i == 1) ? y : z);}
    double  operator[](int i) const {return (i == 0) ? x : ((i == 1) ? y : z);}

    TVector3 operator + (const TVector3 &v) const {return TVector3(x + v.x, y + v.y, z + v.z);}
    TVector3 operator - (const TVector3 &v) const {return TVector3(x - v.x, y - v.y, z - v.z);}
    TVector3 operator - ()                  const {return TVector3(-x, -y, -z);}
    TVector3 operator * (double k)          const {return TVector3(x * k, y * k, z * k);}
    TVector3 operator / (double k)          const {return TVector3(x / k, y / k, z / k);}
    TVector3& operator += (const TVector3 &v){x += v.x; y += v.y; z += v.z; return *this;}
};
inline TVector3 operator * (double k, const TVector3 &v){return v * k;}

//! скалярное произведение
inline double dot(const TVector3 &a, const TVector3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
//! векторное произведение (MATLAB Function cross)
inline TVector3 cross(const TVector3 &a, const TVector3 &b)
{
    return TVector3(a.y * b.z - a.z * b.y,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x);
}
//! длина вектора
inline double length(const TVector3 &a)
{
    return sqrt(dot(a, a));
}

/*! \brief Кватернион q = [q0 q1 q2 q3], q0 - скалярная часть
 *
 * Умножение - по Гамильтону. Кватернион задает поворот
 * из связанной СК в земную (NED): v_e = q * v_b * q~
 * \ingroup math
*/
class TQuaternion
{
public:
    TQuaternion(double q0_ = 1.0, double q1_ = 0.0, double q2_ = 0.0, double q3_ = 0.0)
    {
        q0 = q0_;
        q1 = q1_;
        q2 = q2_;
        q3 = q3_;
    }
    //! кватернион из скаляра и вектора
    TQuaternion(double s, const TVector3 &v)
    {
        q0 = s;
        q1 = v.x;
        q2 = v.y;
        q3 = v.z;
    }
    double q0;
    double q1;
    double q2;
    double q3;

    //! векторная часть
    TVector3 vec() const {return TVector3(q1, q2, q3);}

    TQuaternion operator + (const TQuaternion &p) const {return TQuaternion(q0 + p.q0, q1 + p.q1, q2 + p.q2, q3 + p.q3);}
    TQuaternion operator * (double k)             const {return TQuaternion(q0 * k, q1 * k, q2 * k, q3 * k);}
};

/*! Произведение Гамильтона p*q
 * (блоки "p multiply q" / "p rotiply q" библиотеки QuaternionLib)
*/
inline TQuaternion mult(const TQuaternion &p, const TQuaternion &q)
{
    return TQuaternion(p.q0 * q.q0 - p.q1 * q.q1 - p.q2 * q.q2 - p.q3 * q.q3,
                       p.q0 * q.q1 + p.q1 * q.q0 + p.q2 * q.q3 - p.q3 * q.q2,
                       p.q0 * q.q2 - p.q1 * q.q3 + p.q2 * q.q0 + p.q3 * q.q1,
                       p.q0 * q.q3 + p.q1 * q.q2 - p.q2 * q.q1 + p.q3 * q.q0);
}
//! сопряженный кватернион (блок "q(Conj)")
inline TQuaternion conj(const TQuaternion &q)
{
    return TQuaternion(q.q0, -q.q1, -q.q2, -q.q3);
}
//! норма кватерниона
inline double norm(const TQuaternion &q)
{
    return sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
}
//! нормализация кватерниона (подсистема "Quaternion Normalization")
inline TQuaternion normalize(const TQuaternion &q)
{
    double n = norm(q);
    return TQuaternion(q.q0 / n, q.q1 / n, q.q2 / n, q.q3 / n);
}
//! поворот вектора из связанной СК в земную: q*v*q~ (блок "Body to Earth")
inline TVector3 bodyToEarth(const TQuaternion &q, const TVector3 &v)
{
    return mult(mult(q, TQuaternion(0.0, v)), conj(q)).vec();
}
//! поворот вектора из земной СК в связанную: q~*v*q (блок "Earth to Body")
inline TVector3 earthToBody(const TQuaternion &q, const TVector3 &v)
{
    return mult(mult(conj(q), TQuaternion(0.0, v)), q).vec();
}
/*! Производная кватерниона q' = 1/2 * q * [0, w]
 * (подсистема "Quaternion dot")
 * \param q - кватернион ориентации
 * \param w - угловая скорость в связанной СК [p q r], рад/с
*/
inline TQuaternion quatDot(const TQuaternion &q, const TVector3 &w)
{
    return mult(q, TQuaternion(0.0, w)) * 0.5;
}
/*! Углы Эйлера 3-2-1 в кватернион (блок "Body 3-2-1 to Q", OS4dynamics_01_init.m)
 * \param roll  - крен  (phi),   рад
 * \param pitch - тангаж(theta), рад
 * \param yaw   - курс  (psi),   рад
*/
inline TQuaternion euler321ToQ(double roll, double pitch, double yaw)
{
    double c1 = cos(roll / 2.0),  s1 = sin(roll / 2.0);
    double c2 = cos(pitch / 2.0), s2 = sin(pitch / 2.0);
    double c3 = cos(yaw / 2.0),   s3 = sin(yaw / 2.0);
    return TQuaternion( s1 * s2 * s3 + c1 * c2 * c3,
                       -c1 * s2 * s3 + s1 * c2 * c3,
                        s1 * c2 * s3 + c1 * s2 * c3,
                       -s1 * s2 * c3 + c1 * c2 * s3);
}
/*! Кватернион в углы Эйлера 3-2-1 (блок "Q to Body 3-2-1")
 * \return вектор [крен, тангаж, курс], рад
*/
inline TVector3 qToEuler321(const TQuaternion &q)
{
    double sp = 2.0 * (q.q0 * q.q2 - q.q3 * q.q1);
    if(sp >  1.0) sp =  1.0;
    if(sp < -1.0) sp = -1.0;
    return TVector3(atan2(2.0 * (q.q0 * q.q1 + q.q2 * q.q3), 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2)),
                    asin(sp),
                    atan2(2.0 * (q.q0 * q.q3 + q.q1 * q.q2), 1.0 - 2.0 * (q.q2 * q.q2 + q.q3 * q.q3)));
}
/*! Кватернион из углов 2-3-1 (блок "Body 2-3-1 to Q")
 * \param a - угол вокруг оси y, рад
 * \param b - угол вокруг оси z, рад
 * \param m - угол вокруг оси x, рад
*/
inline TQuaternion euler231ToQ(double a, double b, double m)
{
    double ca = cos(a / 2.0), sa = sin(a / 2.0);
    double cb = cos(b / 2.0), sb = sin(b / 2.0);
    double cm = cos(m / 2.0), sm = sin(m / 2.0);
    return TQuaternion(ca * cb * cm - sa * sb * sm,
                       sa * sb * cm + sm * ca * cb,
                       sa * cb * cm + sb * sm * ca,
                      -sa * sm * cb + sb * ca * cm);
}
//! ограничение значения (блок Saturation)
inline double saturation(double v, double low, double up)
{
    return (v < low) ? low : ((v > up) ? up : v);
}
/*! Одномерная таблица из двух точек с ограничением на краях
 * (блок Lookup Table, Extrapolation = Clip)
*/
inline double lookupClip(double x, double x1, double y1, double x2, double y2)
{
    if(x <= x1) return y1;
    if(x >= x2) return y2;
    return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}

}
#endif // MATH_FUNC_H

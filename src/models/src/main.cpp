#include <iostream>

#include <lambda>

int main()
{
    using namespace lambda;

    {

    matrix<3, 4> u
        (4, 5, 6, 7,
         6, 7, 8, 9,
         8, 9, 10, 11);
    matrix<4, 5> v
        (4, 5, 6, 7, 8,
         5, 6, 7, 8, 9,
         6, 7, 8, 9, 10,
         7, 8, 9, 10, 11);

    matrix<3, 4> K;

    K = u;

    std::cout << "u: " << u << std::endl;
    std::cout << "K: " << K << std::endl;

    auto reduced = rref(v);
    std::cout << "v: " << v << std::endl;
    std::cout << "rref(v): " << reduced << std::endl;

    axis_angle aa({2, 3, 4}, 0.4);
    std::cout << "aa: " << aa << std::endl;

    quaternion q(0, 0.5883484, 0.1961161, 0.7844645);
    aa = q;

    std::cout << "q: " << q << std::endl;
    std::cout << "aa = q: " << aa << std::endl;

    matrix<3, 3> mat = q;
    mat = q;
    std::cout << "mat = q: " << mat << std::endl;

    }
    {

    vector<3> u(1, 2, 3);
    vector<3> v(4, 5, 6);
    std::cout << "u: " << u << std::endl;
    std::cout << "v: " << v << std::endl;
    std::cout << "skew(v): " << skew_symmetric(v) << std::endl;
    std::cout << "cross(u, v): "
        << cross_product(u, v) << std::endl;
    std::cout << "skew(u)*v: " << skew_symmetric(u)*v << std::endl;
    
    }

    lambda::pow(matrix<3, 3>(), 2);

    /* matrix, vector, quaternion tests

    vector<3> x(1.1, 2, 4.5);
    vector<3, int> y(-3, 1, 7);

    std::cout << "u: " << u << std::endl;
    std::cout << "v: " << v << std::endl;
    std::cout << "2v: " << v*2 << std::endl;
    std::cout << "v/2: " << v/2 << std::endl;

    auto w = u*v;
    std::cout << "u*v: " << w << std::endl;

    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "x+y: " << x+y << std::endl;
    std::cout << "x.y: " << inner_product(x, y) << std::endl;
    std::cout << "x*y: " << cross_product(x, y) << std::endl;
    std::cout << "unit vectors: " << unitx << " "
        << unity << " " << unitz << std::endl;

    matrix<2, 2> A(3, 4, 5, 6);
    std::cout << "A: " << A << ", det(A): "
        << det(A) << std::endl;

    matrix<3, 3> B
        (10, 2, 3,
         2, 30, 4,
         3, 4, 50);
    std::cout << "B: " << B << ", det(B): "
        << det(B) << std::endl;

    matrix<4, 4> C
        (6, 2, 3, 4,
         2, 6, 4, 5,
         3, 4, 7, 6,
         4, 5, 6, 7);
    std::cout << "C: " << C << ", det(C): "
        << det(C) << std::endl;

    quaternion<> q(1, 2, 3, 4);
    std::cout << "q: " << q << std::endl;
    std::cout << "q vector: " << q.vec() << std::endl;
    std::cout << "q scalar: " << q.scalar() << std::endl;

    vector<3> k(4, -3, 5);
    std::cout << "k: " << k << std::endl;
    std::cout << "norm(k): " << euclidean_norm(k) << std::endl;
    std::cout << "norm(transpose(k)): "
        << euclidean_norm(transpose(k)) << std::endl;

    std::cout << identity<4>() << std::endl;

    */

    return 0;
}

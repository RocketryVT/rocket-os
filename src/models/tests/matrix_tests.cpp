#include "catch.hpp"
#include <lambda>

TEST_CASE("Quaternion to rotation matrix.", "[matrix]")
{
    lambda::quaternion q(0.5080005, 0.3810004, 0.1270001, 0.7620008);
    lambda::matrix<3, 3> m = q;

    REQUIRE(m(0, 0) == Approx(-0.1935484));
    REQUIRE(m(0, 1) == Approx(-0.6774194));
    REQUIRE(m(0, 2) == Approx(0.7096774));
    REQUIRE(m(1, 0) == Approx(0.8709677));
    REQUIRE(m(1, 1) == Approx(-0.4516129));
    REQUIRE(m(1, 2) == Approx(-0.1935484));
    REQUIRE(m(2, 0) == Approx(0.4516129));
    REQUIRE(m(2, 1) == Approx(0.5806451));
    REQUIRE(m(2, 2) == Approx(0.6774194));
    REQUIRE(lambda::det(m) == Approx(1));
}

TEST_CASE("Throw out of bounds exception.", "[matrix]")
{
    lambda::matrix<3, 4> m;

    REQUIRE_THROWS_WITH( m.at(45, 3),
        "Cannot access element (45, 3) of 3x4 matrix");
}

TEST_CASE("Matrix augmentation.", "[matrix]")
{

    lambda::matrix<3, 3> A(3.4,  2.1, -5.0,
                           7.2, -9.1, -4.0,
                           0.8,  4.2,  4.1);

    lambda::matrix<3, 2> x(4.5,  6.3,  8.2,
                           7.0,  2.5, -9.1);

    auto B = lambda::augment(A, x);

    REQUIRE( B(0, 0) == Approx(3.4) );
    REQUIRE( B(0, 1) == Approx(2.1) );
    REQUIRE( B(0, 2) == Approx(-5.0) );
    REQUIRE( B(1, 0) == Approx(7.2) );
    REQUIRE( B(1, 1) == Approx(-9.1) );
    REQUIRE( B(1, 2) == Approx(-4.0) );
    REQUIRE( B(2, 0) == Approx(0.8) );
    REQUIRE( B(2, 1) == Approx(4.2) );
    REQUIRE( B(2, 2) == Approx(4.1) );

    REQUIRE( B(0, 3) == Approx(4.5) );
    REQUIRE( B(0, 4) == Approx(6.3) );
    REQUIRE( B(1, 3) == Approx(8.2) );
    REQUIRE( B(1, 4) == Approx(7.0) );
    REQUIRE( B(2, 3) == Approx(2.5) );
    REQUIRE( B(2, 4) == Approx(-9.1) );
}

TEST_CASE("Matrix inverse", "[matrix_inverse]")
{
    lambda::matrix<5, 5> m(1, -14, 11,  -6,  -5,
                           6,  -3, 10,  -8, -10,
                          -2,  11,  9, -11,  11,
                           4,  12, -6,  -1,   7,
                          -5,   4,  6,  -4, -10);

    auto inv = lambda::inverse(m);

    lambda::matrix<5, 5> ref(
       -0.13800100021433165,
        0.15306137029363434,
        0.02023290705151104, 
       -0.09784953918696863,
       -0.13029934986068442,
       -0.16069157676644996,
        0.08814746017003644,
        0.04675287561620347,
       -0.11310995213259985,
       -0.03555047510180753,
       -0.3703079231263842,
        0.2353218546831464,
        0.14313067085804101,
       -0.43796527827391585,
       -0.19929984996785025,
       -0.46860041437450883,
        0.2519825676930771,
        0.11695363292134028,
       -0.5091090948060298,
       -0.24540973065656926,
       -0.030020718725441167,
       -0.0008716153461456026,
        0.04768164606701436,
       -0.05545474030149318,
       -0.07048653282846325);

    for (size_t i = 0; i < 25; ++i)
    {
        REQUIRE( inv[i] == Approx(ref[i]) );
    }
}

TEST_CASE("Inverse of non-invertible matrix.", "[matrix_inverse]")
{
    using Catch::Matchers::Contains;

    lambda::matrix<2, 2> m1(1, 2, 2, 4);
    REQUIRE_THROWS_WITH(lambda::inverse(m1), Contains("Cannot invert matrix"));

    lambda::matrix<3, 3> m2(1, 2, 3, 4, 5, 6, 1, 2, 3);
    REQUIRE_THROWS_WITH(lambda::inverse(m2), Contains("Cannot invert matrix"));
}

TEST_CASE("Matrix power calculation.", "[matrix]")
{
    lambda::matrix<3, 3> m(3.4,  2.1, -5.0,
                           7.2, -9.1, -4.0,
                           0.8,  4.2,  4.1);

    lambda::matrix<3, 3> identity = lambda::pow(m, 0);
    lambda::matrix<3, 3> copy = lambda::pow(m, 1);
    auto m2 = m^2;
    auto m5 = m^5;

    REQUIRE(identity(0, 0) == 1);
    REQUIRE(identity(1, 1) == 1);
    REQUIRE(identity(2, 2) == 1);
    REQUIRE(identity(0, 1) == 0);
    REQUIRE(identity(0, 2) == 0);
    REQUIRE(identity(1, 0) == 0);
    REQUIRE(identity(1, 2) == 0);
    REQUIRE(identity(2, 0) == 0);
    REQUIRE(identity(2, 1) == 0);

    REQUIRE(copy(0, 0) == Approx(3.4));
    REQUIRE(copy(0, 1) == Approx(2.1));
    REQUIRE(copy(0, 2) == Approx(-5));
    REQUIRE(copy(1, 0) == Approx(7.2));
    REQUIRE(copy(1, 1) == Approx(-9.1));
    REQUIRE(copy(1, 2) == Approx(-4.0));
    REQUIRE(copy(2, 0) == Approx(0.8));
    REQUIRE(copy(2, 1) == Approx(4.2));
    REQUIRE(copy(2, 2) == Approx(4.1));

    REQUIRE(m2(0, 0) == Approx(22.68));
    REQUIRE(m2(0, 1) == Approx(-32.97));
    REQUIRE(m2(0, 2) == Approx(-45.9));
    REQUIRE(m2(1, 0) == Approx(-44.24));
    REQUIRE(m2(1, 1) == Approx(81.13));
    REQUIRE(m2(1, 2) == Approx(-16));
    REQUIRE(m2(2, 0) == Approx(36.24));
    REQUIRE(m2(2, 1) == Approx(-19.32));
    REQUIRE(m2(2, 2) == Approx(-3.99));

    REQUIRE(m5(0, 0) == Approx(-17469.7));
    REQUIRE(m5(0, 1) == Approx(22338.6));
    REQUIRE(m5(0, 2) == Approx(7241.08));
    REQUIRE(m5(1, 0) == Approx(43169.4));
    REQUIRE(m5(1, 1) == Approx(-83500.3));
    REQUIRE(m5(1, 2) == Approx(-4272.05));
    REQUIRE(m5(2, 0) == Approx(-15195));
    REQUIRE(m5(2, 1) == Approx(22031.2));
    REQUIRE(m5(2, 2) == Approx(-2406.84));
}

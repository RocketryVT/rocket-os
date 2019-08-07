#include "catch.hpp"
#include <lambda>

TEST_CASE("Dynamic matrix construction.", "[dynamic-matrix]")
{
    lambda::dynamic_matrix m(3, 4,
        {1,    2,   3, 4,
         5,    6,   7, 8,
         -3.2, 5.6       });

    REQUIRE( m.rows() == 3 );
    REQUIRE( m.columns() == 4 );

    REQUIRE( m.at(0) == 1 );
    REQUIRE( m.at(1) == 2 );
    REQUIRE( m.at(2) == 3 );
    REQUIRE( m.at(3) == 4 );
    REQUIRE( m.at(4) == 5 );
    REQUIRE( m.at(5) == 6 );
    REQUIRE( m.at(6) == 7 );
    REQUIRE( m.at(7) == 8 );
    REQUIRE( m.at(8) == -3.2 );
    REQUIRE( m.at(9) == 5.6 );
    REQUIRE( m.at(10) == 0 );
    REQUIRE( m.at(11) == 0 );

    REQUIRE( m.at(0, 0) == 1 );
    REQUIRE( m.at(0, 1) == 2 );
    REQUIRE( m.at(0, 2) == 3 );
    REQUIRE( m.at(0, 3) == 4 );
    REQUIRE( m.at(1, 0) == 5 );
    REQUIRE( m.at(1, 1) == 6 );
    REQUIRE( m.at(1, 2) == 7 );
    REQUIRE( m.at(1, 3) == 8 );
    REQUIRE( m.at(2, 0) == -3.2 );
    REQUIRE( m.at(2, 1) == 5.6 );
    REQUIRE( m.at(2, 2) == 0 );
    REQUIRE( m.at(2, 3) == 0 );

    lambda::dynamic_matrix n(2, 2);

    REQUIRE( n.rows() == 2 );
    REQUIRE( n.columns() == 2 );
    REQUIRE( n.at(0) == 0 );
    REQUIRE( n.at(1) == 0 );
    REQUIRE( n.at(2) == 0 );
    REQUIRE( n.at(3) == 0 );
}

TEST_CASE("Dynamic matrix arithmetic.", "[dynamic-matrix]")
{
    auto m = lambda::dunitx*3 + lambda::dunitz + 2*lambda::dunity;

    REQUIRE( m == lambda::dynamic_matrix(3, 1, {3, 2, 1}) );
}

TEST_CASE("Throw dynamic matrix out of bounds exception.", "[dynamic-matrix]")
{
    lambda::dynamic_matrix m(3, 4);

    REQUIRE_THROWS_WITH( m.at(45, 3),
        "Cannot access element (45, 3) of 3x4 matrix");
}

TEST_CASE("Throw dynamic matrix multiplication exception.", "[dynamic-matrix]")
{
    lambda::dynamic_matrix m(3, 4), n(5, 9);

    REQUIRE_THROWS_WITH( m*n,
        "Cannot multiply matrices of dimensions 3x4 and 5x9");
}

#include "catch.hpp"
#include <lambda>

TEST_CASE("Test numerical derivative calculation.", "[solve]")
{
    std::function<double(double)> f = [] (double x)
    {
        return (x - 3)*(x - 5);
    };

    std::function<double(double)> dfdx = [] (double x)
    {
        return 2*x - 8;
    };

    REQUIRE( f(0) == Approx(15) );
    REQUIRE( f(3) == Approx(0) );
    REQUIRE( f(5) == Approx(0) );
    REQUIRE( f(7) == Approx(8) );

    for (int x = -100; x < 100; ++x)
    {
        REQUIRE( lambda::ddx(f, x) == Approx(dfdx(x)) );
    }
}

TEST_CASE("Test Newton-Raphson root solving.", "[solve]")
{
    std::function<double(double)> f = [] (double x)
    {
        return (x - 3)*(x - 5);
    };

    REQUIRE( lambda::newton_raphson(f, 3.5) == Approx(3) );    
    REQUIRE( lambda::newton_raphson(f, 4.5) == Approx(5) );
    REQUIRE( lambda::newton_raphson(f, 4) == Approx(5) );
}

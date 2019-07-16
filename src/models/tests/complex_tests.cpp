#include "catch.hpp"
#include <lambda>

TEST_CASE("Basic complex number operations.", "[complex]")
{
    lambda::complex c(3, 4);
    auto conj = lambda::conj(c);
    
    REQUIRE(c.real() == 3);
    REQUIRE(c.imag() == 4);
    REQUIRE(lambda::arg(c) == Approx(0.927295));
    REQUIRE(lambda::norm(c) == Approx(5));
    REQUIRE(conj.real() == 3);
    REQUIRE(conj.imag() == -4);

    c = lambda::complex(4, 5)/lambda::complex(-0.3, 2);

    REQUIRE(c.real() == Approx(2.151589));
    REQUIRE(c.imag() == Approx(-2.322738));

    c *= lambda::complex(0.23, 0.45);

    REQUIRE(c.real() == Approx(1.54010));
    REQUIRE(c.imag() == Approx(0.433985));

    c += lambda::complex(-1, 1);

    REQUIRE(c.real() == Approx(0.54010));
    REQUIRE(c.imag() == Approx(1.433985));
}

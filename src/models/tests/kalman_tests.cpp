#include "catch.hpp"
#include <lambda>

TEST_CASE("Kalman test.", "[basic kalman]")
{
    lambda::kf<5, 3> kf;
}

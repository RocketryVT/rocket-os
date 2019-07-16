#include "catch.hpp"
#include <lambda>

TEST_CASE("Calculate levenshtein distance.", "[levenshtein]")
{
    std::vector<int> x { 4, 5, 6 }, y { 4, 6, 7};
    
    REQUIRE( lambda::levenshtein(x, y) == 2 );

    REQUIRE( lambda::levenshtein("book", "back") == 2 );
    REQUIRE( lambda::levenshtein("discard", "inscribe") == 6 );
    REQUIRE( lambda::levenshtein("lambda", "lamb") == 2 );
    REQUIRE( lambda::levenshtein("wikipedia", "wookiepedia") == 3 );

    std::array<int, 3> a { 4, 5, 6 };
    std::array<uint8_t, 3> b { 4, 6, 7 };

    REQUIRE( lambda::levenshtein(a, b) == 2 );
}

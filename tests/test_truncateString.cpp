#include <gtest/gtest.h>
#include "truncateString.h"

TEST(TruncateStringTest, ShorterStringUnchanged) {
    std::string input = "abc";
    EXPECT_EQ("abc", truncateString(input, 5));
}

TEST(TruncateStringTest, LongerStringTruncated) {
    std::string input = "abcdef";
    EXPECT_EQ("abcd.", truncateString(input, 5));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

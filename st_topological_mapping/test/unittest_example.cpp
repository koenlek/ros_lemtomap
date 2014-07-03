// This is just a basic, example unittest...
// Bring in my package's API, which is what I'm testing

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, testCase1)
{
int a,b,c;
a=1;
b=3;
c=a+b;

ASSERT_EQ(c,4);
}

// Declare another test
TEST(TestSuite, testCase2)
{
int a,b,c;
a=6;
b=3;
c=a+b;

ASSERT_EQ(c,9);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}

#include "Greeter.h"
#include <algorithm>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <string>

// Run: pio test -e host
TEST(Greeter, OutputsHelloSuccessfully) {
  Greeter greeter;
  testing::internal::CaptureStdout();

  greeter.greet();

  std::string lowercaseResult = testing::internal::GetCapturedStdout();
  std::transform(lowercaseResult.begin(), lowercaseResult.end(),
                 lowercaseResult.begin(), ::tolower);
  EXPECT_NE(lowercaseResult.find("hello"), std::string::npos);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#include <gtest/gtest.h>

#include "autocp/landmarks.h"

namespace autocp {
TEST(LandmarksTest, IsEmptyInitially) {
  Landmarks landmarks;
  EXPECT_EQ(0, landmarks.NumLandmarks());
}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

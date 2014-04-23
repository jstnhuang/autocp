#include <gtest/gtest.h>
#include <OGRE/OgreVector3.h>

#include "autocp/landmarks.h"

namespace autocp {
TEST(LandmarksTest, IsEmptyInitially) {
  Landmarks landmarks;
  EXPECT_EQ(0, landmarks.NumLandmarks());
}

TEST(LandmarksTest, NullLandmarkNotCounted) {
  Landmarks landmarks;
  landmarks.UpdateHead(NULL);
  EXPECT_EQ(0, landmarks.NumLandmarks());
}

TEST(LandmarksTest, NonNullLandmarksCounted) {
  Landmarks landmarks;
  Ogre::Vector3 head_position(0, 0, 1.5);
  landmarks.UpdateHead(&head_position);
  landmarks.UpdateHeadWeight(0.25);
  EXPECT_EQ(1, landmarks.NumLandmarks());
}

TEST(LandmarksTest, ZeroWeightNotCounted) {
  Landmarks landmarks;
  Ogre::Vector3 head_position(0, 0, 1.5);
  landmarks.UpdateHead(&head_position);
  landmarks.UpdateHeadWeight(0);
  EXPECT_EQ(0, landmarks.NumLandmarks());
}

TEST(LandmarksTest, ZeroWeightNotInVector) {
  Landmarks landmarks;
  Ogre::Vector3 head_position(0, 0, 1.5);
  landmarks.UpdateHead(&head_position);
  landmarks.UpdateHeadWeight(0);

  std::vector<Landmark> vector;
  landmarks.LandmarksVector(&vector);
  EXPECT_EQ(0, vector.size());
}

TEST(LandmarksTest, NonZeroWeightInVector) {
  Landmarks landmarks;
  Ogre::Vector3 head_position(0, 0, 1.5);
  landmarks.UpdateHead(&head_position);
  landmarks.UpdateHeadWeight(0.25);

  std::vector<Landmark> vector;
  landmarks.LandmarksVector(&vector);
  EXPECT_EQ(1, vector.size());
}

TEST(LandmarksTest, LandmarksVectorIdempotent) {
  Landmarks landmarks;
  Ogre::Vector3 head_position(0, 0, 1.5);
  landmarks.UpdateHead(&head_position);

  std::vector<Landmark> vector;
  landmarks.LandmarksVector(&vector);
  EXPECT_EQ(0, vector.size());

  landmarks.UpdateHeadWeight(0.25);
  landmarks.LandmarksVector(&vector);
  EXPECT_EQ(1, vector.size());

  landmarks.LandmarksVector(&vector);
  EXPECT_EQ(1, vector.size());
}

TEST(LandmarksTest, AddSegmentedObjectsToVector) {
  Landmarks landmarks;
  std::vector<Ogre::Vector3> segmented_objects;
  segmented_objects.push_back(Ogre::Vector3(0, 0, 0));
  segmented_objects.push_back(Ogre::Vector3(0, 0, 1));
  landmarks.UpdateSegmentedObjects(segmented_objects);
  landmarks.UpdateSegmentedObjectWeight(1);

  EXPECT_EQ(2, landmarks.NumLandmarks());
  std::vector<Landmark> vector;
  landmarks.LandmarksVector(&vector);
  EXPECT_EQ(2, vector.size());
}

TEST(LandmarksTest, SegmentedObjectsSplitWeight) {
  Landmarks landmarks;
  std::vector<Ogre::Vector3> segmented_objects;
  segmented_objects.push_back(Ogre::Vector3(0, 0, -1));
  segmented_objects.push_back(Ogre::Vector3(0, 0, -2));
  landmarks.UpdateSegmentedObjects(segmented_objects);
  landmarks.UpdateSegmentedObjectWeight(0.5);

  Ogre::Vector3 head_position(0, 0, 1);
  landmarks.UpdateHead(&head_position);
  landmarks.UpdateHeadWeight(0.5);

  auto center = landmarks.Center();
  EXPECT_EQ(Ogre::Vector3(0, 0, -0.25), center);
}

TEST(LandmarksTest, CenterComputedByWeight) {
  Landmarks landmarks;
  Ogre::Vector3 head_position(0, 0, 1);
  landmarks.UpdateHead(&head_position);
  landmarks.UpdateHeadWeight(0.25);
  Ogre::Vector3 head_focus_position(0, 0, 2);
  landmarks.UpdateHeadFocus(&head_focus_position);
  landmarks.UpdateHeadFocusWeight(0.75);

  auto center = landmarks.Center();
  EXPECT_EQ(Ogre::Vector3(0, 0, 1.75), center);
}

TEST(LandmarksTest, ZeroWeightNotInCenter) {
  Landmarks landmarks;
  Ogre::Vector3 head_position(0, 0, 1);
  landmarks.UpdateHead(&head_position);
  landmarks.UpdateHeadWeight(0.25);
  Ogre::Vector3 head_focus_position(0, 0, 2);
  landmarks.UpdateHeadFocus(&head_focus_position);
  landmarks.UpdateHeadFocusWeight(0);

  auto center = landmarks.Center();
  EXPECT_EQ(Ogre::Vector3(0, 0, 1), center);
}

TEST(LandmarksTest, MetricComputedByWeight) {
  Landmarks landmarks;
  Ogre::Vector3 head_position(0, 0, 1);
  landmarks.UpdateHead(&head_position);
  landmarks.UpdateHeadWeight(0.25);
  Ogre::Vector3 head_focus_position(0, 0, 2);
  landmarks.UpdateHeadFocus(&head_focus_position);
  landmarks.UpdateHeadFocusWeight(0.75);

  auto z_metric = [&] (const Ogre::Vector3& point) -> float {
    return point.z;
  };
  EXPECT_FLOAT_EQ(1.75, landmarks.ComputeMetric(z_metric));
}

TEST(LandmarksTest, ZeroWeightNotInMetric) {
  Landmarks landmarks;
  Ogre::Vector3 head_position(0, 0, 1);
  landmarks.UpdateHead(&head_position);
  landmarks.UpdateHeadWeight(0.25);
  Ogre::Vector3 head_focus_position(0, 0, 2);
  landmarks.UpdateHeadFocus(&head_focus_position);
  landmarks.UpdateHeadFocusWeight(0);

  auto z_metric = [&] (const Ogre::Vector3& point) -> float {
    return point.z;
  };
  EXPECT_FLOAT_EQ(1, landmarks.ComputeMetric(z_metric));
}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

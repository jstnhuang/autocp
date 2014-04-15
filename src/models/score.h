/*
 * Represents the score of a camera pose. It contains all the components of the
 * score as well as the final score, for debugging purposes.
 */

#ifndef SCORE_H
#define SCORE_H

namespace autocp {
struct Score {
  float visibility;
  float centering;
  float orthogonality;
  float zoom;
  float travel;
  float crossing;
  float score;
 public:
  std::string toString() const {
    char buffer[80];
    snprintf(buffer, 80, "v: %.2f, c: %.2f, o: %.2f, z: %.2f, t: %.2f, c: %.2f = %.2f",
             visibility, centering, orthogonality, zoom, travel, crossing, score);
    return std::string(buffer);
  }
};
}

#endif

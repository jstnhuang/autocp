# Automatic Camera Placement
AutoCP is a display plugin for pr2_interactive_manipulation. Its goal is to automatically position the 3D camera, so that the user can focus on manipulation tasks without having to manually set the camera view. AutoCP is designed specifically for manipulation, and not for navigation.

## Overview
The task is to find a camera position $p$ and focus point $f$ where $p, f \in \mathbb{R}^3$ that maximizes utility to the user according to a score function, $s: \mathbb{R}^3 \times \mathbb{R}^3 \to [0, 1]$. $(p, f)$ together are referred to as a _camera pose_. To accomplish this, we need to define the score function, and we need to specify how to optimize over the camera poses.

## Score function
The bulk of our effort comes from designing the score function. There are several properties of a camera pose that we consider. For each property, we have a hypothesis or multiple hypotheses on what maximizes user utility.

### Landmarks
We define certain points to be _landmarks_. Examples of landmarks include the left and right gripper positions, the point at which the head is looking (aka the _head focus point_), and the locations of segmented objects.

Landmarks may appear or disappear, and they don't necessarily have to map to a physical object. For example, the center of a point cloud could be considered a landmark.

Landmarks can have different importances relative to each other. Their importance can change over time.

*Variants:*

 - Equal, fixed weights on all landmarks
 - If a landmark is associated with something that can be controlled, then the landmark's weight increases in importance when the control is being used. Over time, its weight decays back to its original value.

A variant will be picked empirically and intuitively.
 
### Score formula
Below we describe properties of camera poses and our hypotheses on what makes them useful. Each property is a function $f_i: \mathbb{R}^3 \times \mathbb{R}^3 \to [0, 1]$, where the input is a camera pose. We assume we have knowledge of the current camera pose as well as the locations and weights of the landmarks.

Each property is associated with a weight $w_i$ such that $\sum_i w_i = 1$. The final score of a pose $p$ is $s(p) = \sum_i{w_i f_i(p)}$.
 
### Visibility of landmarks
A landmark is _visible_ if it is onscreen and not occluded by other objects. If a landmark $l$ is visible from $p$, then it gets a visibility score of 1, and 0 otherwise. The visibility score overall is the weighted average score for all landmarks.

**Hypothesis:** a camera pose that keeps the landmarks visible is useful.

### Closeness to landmarks
The distance score for a landmark is 0 if the distance to the landmark is less than a minimum value. If the distance is exactly the minimum value, the score is 1. The score decreases linearly such that the score is 0 at or above a maximum value.

**Hypothesis:** a camera pose that is close to the landmarks is useful, subject to minimum and maximum distance constraints.

### Marker orthogonality
A typical interactive marker has arrows to move in some coordinate direction, as well as rings to rotate the marker around some axis. It's hard to judge depth properly when moving a marker directly towards or away from the camera. We believe that a camera pose orthogonal to the direction the interactive marker is moving is more useful.

When an interactive marker is being used, we compute the vector between the camera pose and the marker, as well as the vector of the marker's movement. The information score is $\mathrm{abs}(\cos(\alpha))$, where $\alpha$ is the angle between the two vectors.

**Hypothesis:** when an interactive marker is being used, an orthogonal camera pose is useful.

### Smoothness of camera movement
Our hypothesis is that neither large nor rapid changes in camera pose are good for utility. There are two main modes for making the camera movement smooth.

#### Smoothing 1
Smoothing 1 is some intuitively and empirically tuned combination of the following:

**Penalizing distance**
The way we "penalize" or "add costs" is actually by making the smoothness score low (close to 0), which makes the overall score lower. We can penalize camera poses that are far away from the current camera pose, both in terms of Euclidean distance, and in terms of the change in the camera angle.

**Regulating speed**
When the system finds the best camera pose, instead of moving the camera to that pose directly, that pose becomes the *target*. Each frame, the camera interpolates its current position and focus point to match the target. In the meantime, the target may change.

**Score threshold**
It could be the case that the system finds better and better poses, but the poses are in very different directions. In that case, the camera will slowly move towards one pose, then move towards another one, and so on. This can be jarring and only lead to very slight improvements in score. To combat that, we can add a percentage threshold, by which a pose must improve on the current target's score in order to become the new target.

**Crossing the line**
Suppose the current camera pose is facing the robot, and the user moves one of the grippers to the right. If, during the interaction, the camera pose changes to a position to the back of the robot, the user is now moving the gripper to the left. A similar situation can arise with forward/backward controls. We can avoid this situation by penalizing it.

It's possible that there is a lot of overlap between all these methods of making the camera movement smooth.

**Hypotheses (smoothing 1):**

 - Adding a cost for movement in terms of Euclidean distance is useful.
 - Adding a cost for movement in terms of change in camera angle is useful.
 - Regulating the speed at which the camera can move is useful.
 - Penalizing movements that don't improve on the current score by some threshold is useful.
 - Penalizing movements which result in "crossing the line" is useful.

#### Smoothing 2
**Moving only when there is no interaction**
When the camera pose changes while the user is using an interactive marker, this may lead the user to make accidental, unpredictable movements with the marker. One possible solution is to freeze the camera pose when the user is interacting with a marker. When the user stops interacting with the marker, the camera can move anywhere it wants. This eliminates the need to penalize distance, threshold on the score, or avoid crossing the line. It also eliminates the marker orthogonality property we described above.

*Variants:* 

 - Incorporate terms that apply to the currently-used control like orthogonality and crossing the line. However, since no control is being used, remember the last control that was used.
 - Incorporate other smoothness terms like distance penalty and score thresholding anyway.

A variant will be picked intuitively and empirically.
 
**Hypothesis:** only moving when there is no interaction is useful.

## Optimization
The optimization is performed by evaluating a fixed set of points at varying distances around each landmark, as well as around the weighted center of the landmarks. At each position, the camera is oriented towards the landmark. For performance reasons, we cannot evaluate each point every frame. Instead, we evaluate a small sample of positions every frame.

The position with the highest score becomes the new target, if its score is higher than the target position's score according to the score threshold.

## Evaluation
The plan is to do an early evaluation on the following table of combinations:

| Properties                              | No smoothing | Smoothing 1 | Smoothing 2 |
| --------------------------------------- | ------------ | ----------- | ----------- |
| Visibility                              |              |             |             |
| Visibility + closeness                  |              |             |             |
| Orthogonality                           |              |             |             |
| Visibility + closeness + orthogonality  |              |             |             |


> Written with [StackEdit](https://stackedit.io/).
# Automatic Camera Placement
AutoCP is a display plugin for pr2_interactive_manipulation. Its goal is to automatically position the 3D camera, so that the user can focus on manipulation tasks without having to manually set the camera view. AutoCP is designed specifically for manipulation, and not for navigation.

## Overview
The task is to find a camera viewpoint, $v=(p, f)$, where $p$ represents a camera position, $f$ represents a focus point (a.k.a. "look-at" point), and where $p, f \in \mathbb{R}^3$. We want to present the viewpoint that maximizes utility to the user according to a score function, $s: \mathbb{R}^3 \times \mathbb{R}^3 \to [0, 1]$. To accomplish this, we need to define the score function, and we need to specify how to optimize over the viewpoints.

### Viewpoint representation
One issue with our chosen representation is that there are multiple possible "look-at" points, which can all represent the same direction. Other ways of representing orientation, such as Euler angles or quaternions, don't have this problem. In practice, this doesn't become a problem for us. For example, we don't needlessly optimize over multiple look-at points which all represent the same direction.

One benefit of our representation is that it's more interpretable. Using a look-at point, it is easy to tell at a glance what a viewpoint represents. Another benefit is that a look-at point fits well with our notion of _landmarks_, described below. Finally, a look-at point can be thought of as a proxy for what the user would like to focus on. A look-at point gives us a term we can use to try and match the user's intent directly.

## Score function
The bulk of our effort comes from designing the score function. There are several properties of a viewpoint that we consider. For each property, we have a hypothesis or multiple hypotheses on what maximizes user utility.

### Landmarks
We define certain points to be _landmarks_. Examples of landmarks include the left and right gripper positions, the point at which the head is looking (a.k.a. the _head focus point_), and the locations of objects involved in manipulation.

Landmarks may be added or removed over time, and they don't necessarily have to map to a physical object. For example, the center of a point cloud could be considered a landmark.

Landmarks can have different weights relative to each other. Their weights can change over time.

Formally, let $L$ be the set of landmarks, such that $l \in \mathbb{R}^3$ for all $l \in L$. The weight of $l$ is $w_L(l)$, subject to the constraint that $\sum_{l \in L} w_L(l) = 1$.

*Variants:*

 - Equal, fixed weights on all landmarks.
 - If a landmark is associated with something that can be controlled, then the landmark's weight increases in importance when the control is being used. Over time, its weight decays back to its original value.
 - Interactive markers can be shown and hidden by the user. If a landmark is associated with a marker, e.g., the grippers, then the landmarks are added and removed with the markers.
 
### Score formula
Below, we describe the properties of a viewpoint which we have incorporated into our score function, as well as our hypotheses on what makes them useful, and the metric we use to measure them. Each metric is a function $f_i: \mathbb{R}^3 \times \mathbb{R}^3 \to [0, 1]$, where the input is a viewpoint. Each property is associated with a weight $w_S(i)$ such that $\sum_i w_S(i) = 1$. The final score of a viewpoint $v$ is $s(v) = \sum_i{w_S(i) f_i(v)}$.

We also assume we have access to global state information as part of the score the computation. The precise information needed for each property will be described below.
 
### Visibility of landmarks
A landmark is _visible_ if it is onscreen and not occluded by other objects.

**Hypothesis:** a viewpoint that keeps the landmarks visible is useful.

**Metric**
If a landmark $l$ is visible from $v$, then it gets a visibility score of 1, and 0 otherwise. The visibility score overall is the weighted average score for all landmarks.

Formally, let $L_v = \{l \mid l \in L \text{ and } l \text{ is visible from } v\}$. Then,

$f_i(v) = \sum_{l \in L_v} w_L(l)$

### Closeness to landmarks
**Hypothesis:** a viewpoint that is close to the landmarks is useful, subject to minimum and maximum distance constraints.

**Metric**
Let $d_\mathrm{min}$ and $d_\mathrm{max}$ be some minimum and maximum distances. Given a landmark $l$ and viewpoint $v=(p, f)$, let $d(l, v) = ||l - p||$ be the Euclidean distance between the landmark and the viewpoint. Then the closeness score is:

$\mathrm{distanceScore}(l, v) =
 \begin{cases}
 0 & \text{if } d(l, v) < d_\mathrm{min} \\
 1 - \frac{d(l, v) - d_\mathrm{min}}{d_\mathrm{max} - d_\mathrm{min}} & \text{if } d_\mathrm{min} \leq d(l, v) \leq d_\mathrm{max} \\
 0 & \text{if } d(l, v) > d_\mathrm{max}
 \end{cases}
$

$f_i(v) = \sum_{l \in L} w_L(l) \cdot \mathrm{distanceScore}(l, v)$

### View angle
A typical interactive marker has arrows to move in some coordinate direction, as well as rings to rotate the marker around some axis. It's hard to judge depth properly when moving an arrow marker directly towards or away from the camera. It's also hard to control a rotation when the viewpoint is not looking at the ring head-on.

**Hypothesis:** when an interactive marker is being used, a viewpoint positioned sideways to the motion of the marker is useful.

**Metric**
Let $c_p$ be the position of the interactive marker control currently being used by the user, and let $c_d$ be the control's type, where $c_d \in \{\mathrm{X, Y, Z, Pitch, Roll, Yaw}\}$.

Given $v=(p, f)$, let $a = c_p - p$ be the vector from the control to the viewpoint.

When an arrow marker is moved, there is a plane of points that are sideways to the motion of the arrow. When a ring marker is moved, there is a line of points that view the ring head-on. Let $b$ be the projection of $a=(a_x, a_y, a_z)$ on the plane or line sideways to the motion of the marker, determined as follows:

$b =
  \begin{cases}
    (0, a_y, a_z) & \text{if } c_d = \mathrm{X} \\
    (a_x, 0, a_z) & \text{if } c_d = \mathrm{Y} \\
    (a_x, a_y, 0) & \text{if } c_d = \mathrm{Z} \\
    (a_x, 0, 0) & \text{if } c_d = \mathrm{Roll} \\
    (0, a_y, 0) & \text{if } c_d = \mathrm{Pitch} \\
    (0, 0, a_z) & \text{if } c_d = \mathrm{Yaw}
  \end{cases}
$

Finally, the score is the cosine of the angle between $a$ and $b$:
$f_i(v) = \left|\frac{a \cdot b}{||a|| \cdot ||b||}\right|$

### Distance from current position
**Hypothesis**: moving the camera shorter distances or not at all is more useful.

**Metric**
Let $t_\mathrm{max}$ be maximum distance, and let $v_\mathrm{current}$ be the current viewpoint being shown to the user.

Let $d = ||v_\mathrm{current} - v||$ be the Euclidean distance between the landmark and the viewpoint. Then the distance score is:

$f_i(v) =
 \begin{cases}
 1 - \frac{d}{t_\mathrm{max}} & \text{if } d \leq t_\mathrm{max} \\
 0 & \text{if } d > t_\mathrm{max}
 \end{cases}
$

## Smoothness of camera movement
Outside of our score function, another consideration for the system is the smoothness of the camera movement. We have two mechanisms for controlling this: regulating speed, and the score threshold.

**Hypotheses:**
 - Regulating the speed at which the camera can move is useful.
 - Penalizing movements that don't improve on the current score by some threshold is useful.

### Regulating speed
When the system finds the best viewpoint, instead of moving the camera to that viewpoint directly, that viewpoint becomes the *target*. Each frame, the camera's position and focus point moves at a fixed speed towards the target. In the meantime, the target may change.

### Score threshold
It may not be worth moving to a new position if the gain in score is small. To account for this, we can say that a viewpoint must improve on the current position's score by at least some percentage.

There may be overlap between the three different methods of smoothing the camera motion. In particular, the "distance from current position" part of the score function and the score threshold may end up serving the same purpose. In that case, we can simply set the score threshold to 1.

## Optimization
The optimization is performed by evaluating a fixed set of points at varying distances around each landmark, as well as around the weighted center of the landmarks. At each position, the camera is oriented towards the landmark. The position with the highest score becomes the new target, if its score is higher than the target position's score according to the score threshold.

Checking if a landmark is visible from a viewpoint takes the most amount of time, which limits us from checking a large number of viewpoints per frame. If there are more viewpoints than can be checked in a 30 FPS budget, we randomly sample viewpoints instead.

*Variants*

- Instead of potentially picking a viewpoint each frame, compute scores for all the viewpoints for as many frames as needed, then pick one. This only works if the scene doesn't change too much over the period of time necessary to do this.

## Evaluation
The plan is to do an early, informal evaluation, which will lead us to select a set of variants to try in a user study.

| Properties                              | No smoothing | With movement penalty |
| --------------------------------------- | ------------ | --------------------- |
| Baseline (no modifications)             |              |                       |
| Visibility                              |              |                       |
| Closeness                               |              |                       |
| View angle                              |              |                       |
| Visibility + closeness + orthogonality  |              |                       |
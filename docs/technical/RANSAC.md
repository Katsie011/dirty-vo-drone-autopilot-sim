# The "BS Detector" — Understanding RANSAC

### **The Problem: The "Drift" of Dirty Data**

In our initial LK Flow POC, the camera "believed" everything it saw. If a shadow moved or a blade of grass swayed, the odometry interpreted that as the drone moving. In robotics, we call these **outliers**. If we include them in our math, our pose estimation becomes a jittery mess.

### **The Solution: RANSAC (Random Sample Consensus)**

Instead of trying to find a "best fit" for all points, RANSAC is an iterative method that ignores the noise.

#### **The Algorithm in 4 Steps:**

1. **Hypothesize:** Grab a tiny random subset of point matches (e.g., 8 points).
2. **Model:** Calculate the motion (Essential Matrix) that explains those 8 points.
3. **Verify:** Check every other point in the frame. Does it fit this motion? If yes, it’s an **Inlier**.
4. **Repeat:** Do this hundreds of times. The set with the most Inliers wins.

#### **The Math of Success**

To ensure we aren't guessing forever, we calculate the number of iterations () needed to be 99% () sure we found a clean set of points, given an outlier ratio ():

**The Learning:** By using RANSAC, we stop the "tail from wagging the dog." We only trust the points that agree on a single, mathematically consistent camera motion.

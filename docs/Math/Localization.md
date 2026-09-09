# Drone Object Localization

## 1. Detection → Pixel Center

Given a bounding box:

$$
(x_1, y_1, x_2, y_2)
$$

take its center:

$$
u = \frac{x_1 + x_2}{2},
\qquad
v = \frac{y_1 + y_2}{2}
$$

This gives the image pixel used for localization.

---

## 2. Pixel → Camera Ray

Using the camera intrinsics:

$$
x = \frac{u - c_x}{f_x},
\qquad
y = \frac{v - c_y}{f_y}
$$

Form the camera-frame direction vector:

$$
\mathbf{d}_{camera}
=
\begin{bmatrix}
x \\
y \\
1
\end{bmatrix}
$$

where:

- $f_x, f_y$ = focal lengths in pixels
- $c_x, c_y$ = optical center
- $\mathbf{d}_{camera}$ = direction relative to the camera
- $\mathbf{d}_{camera}$ is unitless

---

## 3. Camera Frame → Drone Body Frame

Apply the fixed camera mounting rotation:

$$
\mathbf{d}_{body}
=
R_{camera\ mount}\,
\mathbf{d}_{camera}
$$

This accounts for how the camera is physically mounted on the drone.

---

## 4. Drone Body Frame → ENU World Frame

Get the drone orientation quaternion from:

`/mavros/local_position/pose`

MAVROS provides the ROS-facing orientation in **ENU**:

$$
[x, y, z]
=
[\text{East}, \text{North}, \text{Up}]
$$

Convert the quaternion into the drone rotation matrix:

$$
R_{drone}
$$

Then:

$$
\mathbf{d}_{ENU}
=
R_{drone}\,
\mathbf{d}_{body}
$$

Combined:

$$
\boxed{
\mathbf{d}_{ENU}
=
R_{drone}\,
R_{camera\ mount}\,
\mathbf{d}_{camera}
}
$$

Write the resulting world-frame ray as:

$$
\mathbf{d}_{ENU}
=
\begin{bmatrix}
d_E \\
d_N \\
d_U
\end{bmatrix}
$$

where:

- $d_E$ = east component
- $d_N$ = north component
- $d_U$ = up component
- for a downward-pointing ray, $d_U < 0$

---

## 5. Intersect the Ray With the Ground

Let the drone height above ground be:

$$
h
$$

Treat the drone as locally located at:

$$
\mathbf{p}_0
=
\begin{bmatrix}
0 \\
0 \\
h
\end{bmatrix}
$$

The ray is:

$$
\mathbf{p}(t)
=
\mathbf{p}_0
+
t\,\mathbf{d}_{ENU}
$$

Expanded:

$$
E = t\,d_E
$$

$$
N = t\,d_N
$$

$$
U = h + t\,d_U
$$

The ground is where:

$$
U = 0
$$

So:

$$
0 = h + t\,d_U
$$

which gives:

$$
\boxed{
t = -\frac{h}{d_U}
}
$$

Then:

$$
\boxed{
E = t\,d_E
}
$$

$$
\boxed{
N = t\,d_N
}
$$

These are the target's horizontal offsets from the drone in meters.

---

## 6. East/North Offset → Distance and Bearing

Horizontal distance:

$$
\boxed{
r = \sqrt{E^2 + N^2}
}
$$

Bearing clockwise from north:

$$
\boxed{
\theta = \operatorname{atan2}(E, N)
}
$$

---

## 7. Distance + Bearing → GPS

Use GeographicLib with:

- drone latitude
- drone longitude
- bearing $\theta$
- horizontal distance $r$

GeographicLib returns:

$$
(\text{lat}_{target}, \text{lon}_{target})
$$

---

## Full Pipeline

```text
Detection bounding box
        ↓
Pixel center (u, v)
        ↓
Camera intrinsics
        ↓
d_camera = [x, y, 1]
        ↓
Camera mount rotation
        ↓
d_body
        ↓
MAVROS quaternion / drone rotation
        ↓
d_ENU = [d_E, d_N, d_U]
        ↓
t = -h / d_U
        ↓
East / North offsets
        ↓
Distance + bearing
        ↓
GeographicLib
        ↓
Target GPS
```

## Key Inputs

| Input | Source |
|---|---|
| Bounding box | Vision detector |
| $f_x, f_y, c_x, c_y$ | Camera calibration |
| Camera mount rotation | Physical camera mounting configuration |
| Drone orientation | MAVROS `/mavros/local_position/pose` quaternion |
| Height $h$ | Relative altitude topic |
| Drone GPS | MAVROS GPS data |

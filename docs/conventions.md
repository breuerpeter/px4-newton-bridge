# Conventions
## Rotor Indexing
![Astro rotor indices and directions](assets/diagrams/rotor_indexing.svg)
## Model Structure
 - **World**
    - **Body** (`body_frd`) link via "free joint" (floating base), FRD axes
      - **Rotor 1** (`rotor_1`) link via revolute joint, positive rotation along $z$ axis
      - ...
      - **Rotor 4** (`rotor_4`) link via revolute joint, positive rotation along $z$ axis

## Frames

| Frame     | Symbol          | Origin                         | Axes                                   |
| --------- | --------------- | ------------------------------ | -------------------------------------- |
| Body      | $\mathcal{B}$   | Vehicle center of gravity (CG) |  $x$ forward, $y$ right, $z$ down      |
| Rotor $i$ | $\mathcal{R}_i$ | Rotor CG                       | $x$ forward, $z$ according to rotation |

![Frames](assets/images/astro_urdf_frames.png)
![Rotation axes](assets/images/astro_urdf_rot_axes.png)

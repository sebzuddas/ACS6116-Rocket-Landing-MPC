# ACS6116---Rocket-Landing-MPC-
An assignment set for the ACS6116 - Advanced Control module. The assignment focuses on modelling and controlling a Falcon 9 rocket landing, using constrained and unconstrained model predictive control (MPC) techniques. 

## The Objective 
Model a point mass rocket, and land it at the coordinates $\vec r(k_f)=(0, 0, 0)$.

# The Model
$$
    \begin{bmatrix}
        r_x(k+1)\\
        r_y(k+1)\\
        r_z(k+1)\\
        v_x(k+1)\\
        v_y(k+1)\\
        v_z(k+1)\\
    \end{bmatrix}
    =
    \begin{bmatrix}
       1 & 0 & 0 & T_s & 0 & 0\\
       0 & 1 & 0 & 0 & T_s & 0\\
       0 & 0 & 1 & 0 & 0 & T_s\\
       0 & 0 & 0 & 1 & 0 & 0\\
       0 & 0 & 0 & 0 & 1 & 0\\
       0 & 0 & 0 & 0 & 0 & 1
    \end{bmatrix}
    \begin{bmatrix}
        r_x(k)\\
        r_y(k)\\
        r_z(k)\\
        v_x(k)\\
        v_y(k)\\
        v_z(k)\\
    \end{bmatrix}
    +
    \begin{bmatrix}
        \frac{T_s^2}{2m} & 0 & 0\\
        0 & \frac{T_s^2}{2m} & 0\\
        0 & 0 & \frac{T_s^2}{2m}\\
        T_s & 0 & 0\\
        0 & T_s & 0\\
        0 & 0 & T_s
    \end{bmatrix}
    \begin{bmatrix}
        f_x(k)+w_x\\
        f_y(k)+w_y\\
        f_z(k)-mg\\
    \end{bmatrix}
$$

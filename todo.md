We need to set up a Rocket physics simulator. This will be made in Python.
First we gotta code in the most important physical equations, like the equation of acceleration (a rocket experiences acceleration in all directions every milisecond). Together with the equation of Drag.
Together with this we need to implement a viewer where we can display the data.
Add a function to import rocket motor datasets, or possibly search up already implemented motors.

Using Matplotlib for 2d graphs like altitude over time is a good starting point. Together with using NumPy for the heavy lifting in physics and math.
Panda3D or Ursina are both options for importing CAD models into the visualizations (obj or STL files)
Use a DSV parser to read the Thrust Curves from specific motors

Later add Wind Gusts to make it more realistic so you can see the fin control needing to be active to keep the model straight.
Together with maybe adding sensor noise, so simulate for example a "noisy" barometer, to show the Kalman Filter or a Moving Average to find the actual appoge

Equations:
    Acceleration (a) 
        a = (T-D)/m - g
            T: Thrust from the motor
            D: Drag on the rocket
            m: the changing mass (weight decresses with motor burn)
            g: Gravity (9.82m/s^2 in Denmark, global is 9.81m/s^2)get
    Drag (D)
        D=1/2 * rho * v^2 * C_d * A
            rho (greek letter for air density): Air density (Changes with altitude)
            v: Velocity
            C_d: Drag Coefficient (model rocket standard is usually around 0.45 to 0.75)
            A: Frontal surface area (Tube cross section)
# Bereshit Landing Simulation and Analysis

## Project Overview

This project simulates the landing of the Bereshit spacecraft, designed as part of an Aerospace Engineering course project. The simulation models key parameters such as altitude, vertical speed, horizontal speed, angle, and fuel consumption during the descent.

The simulation uses PID controllers to adjust the vertical speed and angle based on altitude, and it tracks the fuel consumption over time. The output data is stored in a CSV file and visualized using Python's matplotlib and seaborn libraries.

## Features

#### Simulation of Landing: The physics of the landing are modeled, adjusting for fuel consumption, acceleration, and descent speed.

#### PID Controllers: Two PID controllers are used to control the vertical speed and the angle during the descent. 

#### Data Visualization: The simulation data is visualized with 5 plots to help analyze the landing process.

#### CSV Export: The simulation results are saved in a CSV file for further analysis.

## Technical Report:

A Technical Report has been added to analyze the crash of the Bereshit spacecraft, providing a detailed breakdown of the incident. The report also outlines the simulation used to model the landing sequence, the assumptions made during the simulation, and the results obtained from various parameters such as altitude, vertical speed, horizontal speed, angle, and fuel consumption. Through this simulation, we aim to identify the factors contributing to the crash, further explaining both the methodology of our simulation and the insights gained from our results.




## Python Script: plot_landing.py

#### Prerequisites:

Python 3.6 or higher

#### Required Python libraries:

##### pandas

##### matplotlib

##### seaborn

### You can install the required libraries using pip:

pip install pandas matplotlib seaborn

### How to Run the Python Script

Export CSV: First, ensure that you have the landing_report.csv file. This CSV file is generated by running the Java-based landing simulation (Bereshit_101.java).

The CSV file contains the simulation data  including time, vertical speed, horizontal speed, altitude, angle, fuel, and acceleration.

Run the Python Script: Once you have the landing_report.csv file, you can run the Python script to generate the plots. Make sure that the plot_landing.py script is in the same directory as the CSV file.

To run the script, use the following command in your terminal:

python plot_landing.py

View the Results: After running the script, it will generate a plot showing 5 graphs of the simulation's key metrics: altitude, vertical speed, horizontal speed, angle, and fuel. The plot will be saved as 

bereshit_landing.png in the current directory.


## Description of the Plots:

*Altitude vs Time*: Shows the descent of the spacecraft over time.

*Vertical Speed vs Time*: Tracks the vertical speed during descent.

*Horizontal Speed vs Time*: Shows the horizontal speed of the spacecraft.

*Angle vs Time*: Tracks the angle of the spacecraft.

*Fuel vs Time*: Monitors the fuel consumption over time.

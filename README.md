# Introduction

This is a basic electric vehicle simulator written in python. It accepts time and speed coordinates and, at given timesteps, calculates the motor requirements to reach them. It uses a PID loop to determine required motor effort and compensates for rolling resistance and wind resistance. 

# Prerequisites

This program runs on python 3. There are no other requirements. 

# How to Use

Provide the input time,speed coordinates in text and specify the timestamp and object properties. Run with python main.py in the terminal. The results will be output to a CSV file and that will be openned after creation in the default editor. 
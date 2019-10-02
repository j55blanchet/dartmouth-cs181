# Programming Assignment #1
Julien Blanchet &nbsp;&bull;&nbsp; COSC 181  &nbsp;&bull;&nbsp;Fall 2019  

<hr>
Project Link: https://github.com/j55blanchet/dartmouth-cs181
<hr>

## Description

My submission uses a different approach for task 1 & 2 as compared to task 3.

##### Tasks #1 and #2
Given that tasks 1 and 2 asked us to draw shapes with fixed geometry (scaled to a given `radius`), I decided to precompute the geometry and hardcode the sequence of movements. This is admittedly less flexible, but resulted in clean and very readable code. Drawing a trapazoid or a semicircle with the turtlebot simulator can be broken down into a sequences of rotations, translations, or arcs (both at the same time). From testing and reading the documentation online, I found that I could pass nearly any desired linear or angular velocity to the turtle and it'd complete the action within a second. Therefore, but publishing one `cmd_vel` per second in the predetermined sequence of rotations/translations, I could reliably direct the bot on these paths.

The sequence for task #1 is as follows.

##### Task #3



## Evaluation

* Does your program actually work? How well? 
* If it doesn’t work, can you tell why not? What partial successes did you have that deserve partial credit? 

## Allocation of Effort
All work on this project was done individually




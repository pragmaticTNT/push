To Run Default type:

make && ./push -w 10 -h 10 -r 30 -z 0.25 -b 11 -s 0.5 -g k.txt

these are respectively:
w:  width of window
h:  height of window
r:  number of robots
b:  number of boxes
z:  diameter of robots (pretty sure, even though it says radius)
s:  diameter of boxes (same idea)
g:  file storing the goal coordinates

REQUIREMENTS:
    -glfw3
    -box2d
Default behaviour popluates a wxh world with r randomly placed robots and b 
randomly placed boxes. Type:

make
./push -w 10 -h 10 -r 30 -z 0.25 -b 11 -s 0.5 -p C

these are respectively:
w:  width of window
h:  height of window
r:  number of robots
b:  number of boxes
z:  diameter of robots (pretty sure, even though it says radius)
s:  diameter of boxes (same idea)
p:  shape of the boxes (S)quare, (H)exagon, (C)ircle

Press space to unpause. To run Goal File type:

make
./push -g goalFileName.txt

the field is:
g:  goal file storing the goal coordinates (path from the patterns folder) 

Read goal_template_instructions.txt in the patterns folder for more details
regarding the format of the goal files.

REQUIREMENTS:
    -glfw3
    -box2d

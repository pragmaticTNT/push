### Robot Simulator

Default behaviour popluates a **wxl** world with **r** randomly placed robots of diameter **z**  and **b** randomly placed boxes of diameter **s** and shape **p**, type:

'''
make
./push -w 10 -l 10 -r 30 -z 0.25 -b 11 -s 0.5 -p C
'''

these are respectively:
- w:  width of window
- l:  length of window
- r:  number of robots
- b:  number of boxes
- z:  diameter of robots
- s:  diameter of boxes
- p:  shape of the boxes (S)quare, (H)exagon, (C)ircle

Press space to unpause. To run a goal file type:

'''
make
./push -g goalFileName.txt
'''

the field is:
- g:  goal file storing the goal coordinates (path from the patterns folder) 

Read goal_template_instructions.txt in the patterns folder for more details regarding the format.

REQUIREMENTS:
    - glfw3
    - box2d

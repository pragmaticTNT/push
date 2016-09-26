# Robot Simulator

Default behaviour popluates a world with **r** randomly placed robots and **b** randomly placed boxes:

<pre>
make
./push -r 30 -b 11
</pre>

these are respectively:
- r:  number of robots
- b:  number of boxes

Press space to unpause. To run a goal file type:

<pre>
make
./push -g goalFileName.txt
</pre>

the field is:
- g:  goal file storing the goal coordinates (path from the patterns folder) 

Read goal_template_instructions.txt in the patterns folder for more details regarding the format. To generate a goal file from an image: take *my_image.png* or *my_image.jpg* and make it grey-scale. Each pixel of the image is a potential location for a goal so images should be no larger than 1000 x 1000 pixels or the simulator will run very slowly. Run:

<pre>
python goalFileGenerator.py
</pre>

The number of extra boxes and the acceptable darkness of a pixel before it
counts as a goal location can be tweeked in *goalFileGenerator.py*.

To graph the collected data. Run: 
<pre>
python plotData.py
</pre>

###REQUIREMENTS:
    - glfw3
    - box2d

###Trouble Shooting:
If you encounter an error please consult the following:

    - (Using a goal file) Compiles but not reading in data: make sure you have `-g` or `--goalFileName` **before** your goal file name. 

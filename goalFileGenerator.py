import datetime
import os
import os.path

from PIL import Image

TMP_FOLDER  = "templates/"
IMG_FOLDER  = "images/"
TMP_FILE    = TMP_FOLDER + "grid_goal_template_simple.txt"
IMG_EXT     = ".jpg"
FILE_EXT    = ".txt"
GOAL_EXT    = ".tmp"

ADD_BOXES   = 0.1   ## Number of boxes increased by this amount
CLR_ERR     = 10    ## Black is 0, accept small error

## PURPOSE: Read each pixel of an image and determine if a goal
##          should be placed at this (row,col) location. Creates
##          temporary goalfile.
## INPUT:   - name (str): name of image (no extension)
## OUTPUT:  - width (int): of image (in pixels)
##          - height (int): of image (in pixels)
##          - num_goals (int): numbers of lines added to tmp file
## NOTE:    Grayscale images only (may cause file to generate 0 goals)
def gen_goals(name):
    print "===> Generating tmp goal file..."
    img = Image.open(IMG_FOLDER + name + IMG_EXT)
    pix = img.load()
    height, width = img.size
    print "... ", width, "x", height
    num_goal = 0
    tmp_goal = open(name + GOAL_EXT, 'w')
    for col in range(height):
        for row in range(width):
            ## print "Pixel (", row, ',', col, "): ", pix[row, col]
            if pix[row, col] < CLR_ERR:
                goal = str(col) + ' ' + str(row) + '\n'
                tmp_goal.write(goal)
                num_goal += 1
    print "...  Number of goals:", num_goal
    tmp_goal.close()
    return (width, height, num_goal)

## PURPOSE: Cleans up the tmp file created by <gen_goals>
## INPUT:   - name (str): name of tmp goal file (no extensions)
## OUTPUT:  (NONE)
def rmv_goals(name):
    print "===> Removing tmp goal file..."
    tmp_goal = name + GOAL_EXT
    os.remove(tmp_goal)

## PURPOSE: Stiches together the goal template and tmp goal file
##          generated by <gen_goals>
## INPUT:   - name (str): of image (no extensions)
##          - width (int): of image (in pixels)
##          - height (int): of image (in pixels)
##          - num_goals (int): numbers of lines added to tmp file
## OUTPUT:  - sucess (bool): did the template file get generated?
def gen_file(name, rows, cols, num_goals):
    goal_file = name + GOAL_EXT
    if not os.path.isfile(goal_file):
        return False

    new_template = open(TMP_FOLDER + name + FILE_EXT, 'w')

    ## Write contents of grid goal template
    with open(TMP_FILE, 'r') as readfile:
        width = 1
        box_size = 0;
        for line in readfile:
            words = line.split()
            ## Edit: current date
            if len(words) > 1 and words[1] == "Latest":
                day = datetime.date
                line = "# Latest Revision: " + str(day.today()) + "\n"
            ## Edit: settings
            if len(words) > 0 and words[0] != '#':
                if words[0] == "width:":
                    width = float(words[1])
                if words[0] == "boxNum:":
                    # add some percentage more boxes
                    words[1] = str(num_goals + int(ADD_BOXES*num_goals))
                elif words[0] == "boxSize:":
                    box_size = round(width/(rows+1), 2)
                    words[1] = str( box_size )
                elif words[0] == "robotSize:":
                    words[1] = str( box_size*0.75 )
                elif words[0] == "dimGrid:":
                    words[1] = str( rows+1 )
                elif words[0] == "goalError:":
                    words[1] = str( box_size )
                    ## print box_size, "goalError:", words[1]
                words.append("\n");
                line = ' '.join(words)
            new_template.write(line)

    new_template.write('\n')

    ## Write contents of tmp_goal
    with open(goal_file, 'r') as readfile:
        for line in readfile:
            new_template.write(line)
    new_template.close()

    return True

def main ():
    name = raw_input("Enter image name (no extension): ").split()[0]
    rows, cols, num_goals = gen_goals(name)
    if gen_file(name, rows, cols, num_goals):
        print "===> [SUCCESS] Goal file for", name, "generated!"
        rmv_goals(name)
    else:
        print "===> [FAIL] Error occured: \"" + name + IMG_EXT + "\" not found."

if __name__ == "__main__":
    main()

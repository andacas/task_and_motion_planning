import kautham_python_interface as kautham
import ktmpb_python_interface
#from ktmpb_python_interface  import writePath

import xml.etree.ElementTree as ET
from collections import defaultdict

global approach_section
approach_section = defaultdict(lambda: defaultdict(dict))

def APPROACH_SECTION(node, approach_section,info,Line): #management of the action on the task plan
    print("**************************************************************************")
    print("  APPROACH_SECTION ACTION  ")
    print("**************************************************************************")
    action=Line[0]
    rob= Line[1]
    fromLocation = Line[2]
    toLocation = Line[3]
    
    print(action +" "+rob+" "+fromLocation+" "+toLocation)
    try:
        init = approach_section['InitControls']
        goal= approach_section['GoalControls']
        Robot_control=approach_section['Cont']
    except:
        try:
            print ("Tag for "+ action +" "+rob+" "+fromLocation+" "+toLocation+" Not defined in config file \nHence using "+ action +" "+rob+" "+toLocation+" "+fromLocation+" In reverse instead")
            goal = approach_section['InitControls']
            init = approach_section['GoalControls']
            Robot_control=approach_section['Cont']
        except:
            print(action +" "+rob+" "+toLocation+" "+fromLocation+" Not defined in config file ")
    print("Searching path to Move robot " )
    print("Init= ", fromLocation)
    print("Goal= ", toLocation)
    print("Init= ", init)
    print("Goal=", goal)
    #Home=approach_section[rob][fromLocation][toLocation]['Home']
    info.Robot_move_control= Robot_control
    print("Robot control=",Robot_control)
    #Set Robot Control
    kautham.kSetRobControlsNoQuery(node, Robot_control)
    #Set the approach_section query in Kautham
    kautham.kSetQuery(node, init,goal)
    #Solve query
    print("Solving Query")
    kautham.kSetPlannerParameter(node, "_Incremental (0/1)","0") #to assure a fresh GetPath
    path=kautham.kGetPath(node, 1)
    #Write path to taskfile
    if path:
        print("-------- Path found: Moving robot " )
        if info.graspedobject is False:
            info.taskfile.write("\t<Transit>\n")
            
            k = sorted(list(path.keys()))[-1][1]+1 #number of joints
            p = sorted(list(path.keys()))[-1][0]+1 #number of points in the path
            for i in range(p):
                tex=''
                for j in range(0,k):
                    tex=tex + str(path[i,j]) + " "
                ktmpb_python_interface.writePath(info.taskfile,tex)
            info.taskfile.write("\t</Transit>\n")
        else:
            #not sure when this else is need...
            k = sorted(list(path.keys()))[-1][1]+1 #number of joints
            p = sorted(list(path.keys()))[-1][0]+1 #number of points in the path
            for i in range(p):
                tex=''
                for j in range(0,k):
                    tex=tex + str(path[i,j]) + " "
                ktmpb_python_interface.writePath(info.taskfile,tex)

        kautham.kMoveRobot(node, goal)
    else:
        print("**************************************************************************")
        print("Get path Failed! No Move possible, Infeasible Task Plan")
        print("**************************************************************************")
        return False

    return True

def Approach_section_read(action_element): #reading from the tamp configuration file
    for val in action_element.attrib:
        globals()[val] = action_element.attrib[val]

    approach_section = {}

    for el in action_element:
        try:
            globals()[el.tag] = int(el.text)
        except:
            try:
                globals()[el.tag] = [float(f) for f in str(el.text).strip().split()]
            except:
                globals()[el.tag] = str(el.text).strip()
        approach_section.update({el.tag : globals()[el.tag]})
    return approach_section

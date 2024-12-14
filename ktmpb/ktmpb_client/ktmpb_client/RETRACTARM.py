import kautham_python_interface as kautham
import ktmpb_python_interface
import xml.etree.ElementTree as ET
from collections import defaultdict

global retractarm
retractarm = defaultdict(lambda: defaultdict(dict))

def RETRACTARM(node, retractarm, info, Line):
    print("**************************************************************************")
    print("  RETRACT ARM ACTION  ")
    print("**************************************************************************")

    # Expected line format: (RETRACT_ARM MADAR HAND SECTION LOCATION)
    # Adjust indexing based on how you define retract_arm in the plan.
    action = Line[0]    # "retract_arm"
    rob = Line[1]       # Robot name, e.g. MADAR
    hand = Line[2]      # Hand name, e.g. LEFT-HAND
    section = Line[4]   # Section from which we retract
    location = Line[3]  # The location (e.g. SHELF)

    print(action, rob, hand, section, location)

    # Extract from retractarm dictionary
    Robot_control = retractarm['Cont']
    init = retractarm['InitControls']
    goal = retractarm['GoalControls']

    # Set robot control
    kautham.kSetRobControlsNoQuery(node, Robot_control)

    print("Searching path to retract arm")
    print("Init=", init)
    print("Goal=", goal)
    print("Robot Control=", Robot_control)

    kautham.kSetQuery(node, init, goal)
    kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")
    path = kautham.kGetPath(node, 1)

    if path:
        print("-------- Path found: Retracting arm")
        info.taskfile.write("\t<Transit>\n")
        k = sorted(list(path.keys()))[-1][1]+1
        p = sorted(list(path.keys()))[-1][0]+1
        for i in range(p):
            tex=''
            for j in range(k):
                tex = tex + str(path[i,j]) + " "
            ktmpb_python_interface.writePath(info.taskfile, tex)
        info.taskfile.write("\t</Transit>\n")
        kautham.kMoveRobot(node, goal)
    else:
        print("**************************************************************************")
        print("Get path Failed! No Move possible, Infeasible Task Plan")
        print("**************************************************************************")
        return False

    print("Arm retracted")
    return True

def Retractarm_read(action_element):
    retractarm = {}
    for el in action_element:
        try:
            retractarm[el.tag] = [float(f) for f in el.text.strip().split()]
        except:
            retractarm[el.tag] = str(el.text).strip()
    return retractarm

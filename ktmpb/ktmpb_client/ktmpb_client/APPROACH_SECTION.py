import kautham_python_interface as kautham
import ktmpb_python_interface
import xml.etree.ElementTree as ET
from collections import defaultdict

global approach_section
approach_section = defaultdict(lambda: defaultdict(dict))

def APPROACH_SECTION(node, approach_section, info, Line):
    print("**************************************************************************")
    print("  APPROACH-SECTION ACTION  ")
    print("**************************************************************************")
    action = Line[0]
    rob = Line[1]
    hand = Line[2]
    location = Line[3]
    section = Line[4]

    print(f"{action} {rob} {hand} {location} {section}")
    try:
        init = approach_section['InitControls']
        goal = approach_section['GoalControls']
        Robot_control = approach_section['Cont']
    except KeyError:
        print(f"Configuration for {action} {rob} {hand} {location} {section} not defined in config file")
        return False

    print("Moving robot's hand to section")
    print(f"Init: {init}")
    print(f"Goal: {goal}")
    print(f"Robot control: {Robot_control}")

    # Set Robot Control
    kautham.kSetRobControlsNoQuery(node, Robot_control)
    # Set the approach query in Kautham
    kautham.kSetQuery(node, init, goal)
    # Solve query
    print("Solving Query")
    kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")  # Ensure fresh GetPath
    path = kautham.kGetPath(node, 1)

    # Write path to taskfile
    if path:
        print("-------- Path found: Moving robot's hand to section --------")
        info.taskfile.write("\t<Transit>\n")
        k = sorted(list(path.keys()))[-1][1] + 1  # Number of joints
        p = sorted(list(path.keys()))[-1][0] + 1  # Number of points in the path
        for i in range(p):
            tex = ''
            for j in range(0, k):
                tex = tex + str(path[i, j]) + " "
            ktmpb_python_interface.writePath(info.taskfile, tex)
        info.taskfile.write("\t</Transit>\n")
        kautham.kMoveRobot(node, goal)
    else:
        print("**************************************************************************")
        print("Get path Failed! No Approach possible, Infeasible Task Plan")
        print("**************************************************************************")
        return False

    return True

def Approach_section_read(action_element):
    for val in action_element.attrib:
        globals()[val] = action_element.attrib[val]

    approach_section = {}

    for el in action_element:
        try:
            globals()[el.tag] = int(el.text)
        except ValueError:
            try:
                globals()[el.tag] = [float(f) for f in str(el.text).strip().split()]
            except ValueError:
                globals()[el.tag] = str(el.text).strip()
        approach_section.update({el.tag: globals()[el.tag]})
    return approach_section

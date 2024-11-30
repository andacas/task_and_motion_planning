import kautham_python_interface as kautham
import ktmpb_python_interface
import xml.etree.ElementTree as ET
from collections import defaultdict

global approachsections2arms
approachsections2arms = defaultdict(lambda: defaultdict(dict))

def APPROACHSECTIONS2ARMS(node, approachsections2arms, info, Line):
    print("**************************************************************************")
    print("  APPROACH-SECTIONS-2-ARMS ACTION  ")
    print("**************************************************************************")
    action = Line[0]
    rob = Line[1]
    hand1 = Line[2]
    hand2 = Line[3]
    location = Line[4]
    section1 = Line[5]
    section2 = Line[6]

    print(f"{action} {rob} {hand1} {hand2} {location} {section1} {section2}")
    try:
        init = approachsections2arms['InitControls']
        goal = approachsections2arms['GoalControls']
        Robot_control = approachsections2arms['Cont']
    except KeyError:
        try:
            print(f"Tag for {action} {rob} {hand1} {hand2} {location} {section1} {section2} not defined in config file")
            # Attempt to reverse init and goal if not found
            goal = approachsections2arms['InitControls']
            init = approachsections2arms['GoalControls']
            Robot_control = approachsections2arms['Cont']
        except KeyError:
            print(f"Configuration for {action} {rob} {hand1} {hand2} {location} {section1} {section2} not defined in config file")
            return False

    print("Moving robot's both hands to sections")
    print(f"Init= {init}")
    print(f"Goal= {goal}")
    print(f"Robot control= {Robot_control}")

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
        print("-------- Path found: Moving robot's both hands to sections --------")
        if info.graspedobject is False:
            info.taskfile.write("\t<Transit>\n")

            k = sorted(list(path.keys()))[-1][1] + 1  # Number of joints
            p = sorted(list(path.keys()))[-1][0] + 1  # Number of points in the path
            for i in range(p):
                tex = ''
                for j in range(0, k):
                    tex += str(path[i, j]) + " "
                ktmpb_python_interface.writePath(info.taskfile, tex)
            info.taskfile.write("\t</Transit>\n")
        else:
            # Not sure when this else is needed...
            k = sorted(list(path.keys()))[-1][1] + 1  # Number of joints
            p = sorted(list(path.keys()))[-1][0] + 1  # Number of points in the path
            for i in range(p):
                tex = ''
                for j in range(0, k):
                    tex += str(path[i, j]) + " "
                ktmpb_python_interface.writePath(info.taskfile, tex)

        kautham.kMoveRobot(node, goal)
    else:
        print("**************************************************************************")
        print("Get path Failed! No Approach possible, Infeasible Task Plan")
        print("**************************************************************************")
        return False

    return True

def Approachsections2arms_read(action_element):  # Reading from the TAMP configuration file
    for val in action_element.attrib:
        globals()[val] = action_element.attrib[val]

    approachsections2arms = {}

    for el in action_element:
        try:
            globals()[el.tag] = int(el.text)
        except:
            try:
                globals()[el.tag] = [float(f) for f in str(el.text).strip().split()]
            except:
                globals()[el.tag] = str(el.text).strip()
        approachsections2arms.update({el.tag: globals()[el.tag]})
    return approachsections2arms

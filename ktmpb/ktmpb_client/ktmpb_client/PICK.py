import kautham_python_interface as kautham
import ktmpb_python_interface
import xml.etree.ElementTree as ET
from collections import defaultdict

global pick
pick = defaultdict(lambda: defaultdict(dict))

def PICK(node, pick, info, Line):  # Management of the action on the task plan
    print("**************************************************************************")
    print("  PICK ACTION  ")
    print("**************************************************************************")
    action = Line[0]
    rob = Line[1]
    hand = Line[2]
    object_name = Line[3]
    section = Line[4]

    print(f"{action} {rob} {hand} {object_name} {section}")
    
    try:
        init = pick['Regioncontrols']
        grasp_controls = pick['Graspcontrols'][object_name]
        Robot_control = pick['Cont']
        linkIndex = pick['Link']
    except KeyError as e:
        print(f"KeyError: {e}")
        print(f"Configuration for {action} {rob} {hand} {object_name} {section} not defined in config file")
        return False

    # Set Robot Control
    kautham.kSetRobControlsNoQuery(node, Robot_control)
    
    # Iterate through grasp controls
    path_found = False
    for grasp in sorted(grasp_controls.keys()):
        goal = grasp_controls[grasp]
        print(f"Searching path to grasp {object_name} at {section}")
        print(f"Init= {init}")
        print(f"Goal= {goal}")
        print(f"Robot Control= {Robot_control}")
        
        # Set query
        kautham.kSetQuery(node, init, goal)
        print("Solving Query to pick object")
        kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")  # Ensure fresh GetPath
        path = kautham.kGetPath(node, 1)
        
        if path:
            print(f"Path found: Moving to pick {object_name} at {section}")
            info.taskfile.write("\t<Transit>\n")
            k = sorted(list(path.keys()))[-1][1] + 1  # Number of joints
            p = sorted(list(path.keys()))[-1][0] + 1  # Number of points in the path
            for i in range(p):
                tex = ''
                for j in range(k):
                    tex += f"{path[i, j]} "
                ktmpb_python_interface.writePath(info.taskfile, tex)
            info.taskfile.write("\t</Transit>\n")
            kautham.kMoveRobot(node, goal)
            path_found = True
            info.graspControlsUsed = grasp
            break
        else:
            print("**************************************************************************")
            print("Get path Failed! No Pick possible, Infeasible Task Plan")
            print("**************************************************************************")
            continue

    if not path_found:
        print("**************************************************************************")
        print("Pick action failed: No valid grasp path found")
        print("**************************************************************************")
        return False

    # Attach object
    print(f"Picking object {object_name}")
    kautham.kAttachObject(node, rob, linkIndex, object_name)

    return True

def Pick_read(action_element):  # Reading from the TAMP configuration file
    for val in action_element.attrib:
        globals()[val] = action_element.attrib[val]

    pick = {}
    grasp = {}

    for el in action_element:
        try:
            pick[el.tag] = int(el.text)
        except ValueError:
            try:
                pick[el.tag] = [float(f) for f in str(el.text).strip().split()]
            except ValueError:
                pick[el.tag] = str(el.text).strip()
        if el.tag == 'Graspcontrols':  # Variables with multiple entries should be added the same way
            grasp_name = el.get('grasp')
            grasp_control = el.text
            grasp_control = [float(f) for f in grasp_control.split()]
            grasp[grasp_name] = grasp_control
            pick[el.tag] = grasp

    if len(grasp) == 0:
        print('No grasp conf found - This may be a problem')
    else:
        print('grasp = ', grasp)

    return pick

import kautham_python_interface as kautham
import ktmpb_python_interface
import xml.etree.ElementTree as ET
from collections import defaultdict

global path
global pick
pick = defaultdict(lambda: defaultdict(dict))

def PICK(node, pick, info, Line):  # management of the action on the task plan
    print("**************************************************************************")
    print("  PICK ACTION  ")
    print("**************************************************************************")

    action = Line[0]       # "pick"
    rob = Line[1]          # Robot (e.g., MADAR)
    hand = Line[2]         # Hand (e.g., LEFT-HAND)
    object_name = Line[3]  # Object to pick (e.g., glass1)
    section = Line[4]      # Section where the object is (e.g., SHELF_1)

    print(action, rob, hand, object_name, section)

    # Extract necessary info from 'pick' dictionary provided by TAMP config
    obsName = pick['Obj']        # The object name from config (should match object_name)
    robotIndex = pick['Rob']     # Robot index
    linkIndex = pick['Link']     # Link index corresponding to the hand
    init = pick['Regioncontrols']
    Robot_control = pick['Cont']

    # Set robot control
    kautham.kSetRobControlsNoQuery(node, Robot_control)

    path = None
    if 'Graspcontrols' in pick.keys():
        grasp_control = pick['Graspcontrols']
        # Attempt each defined grasp until a feasible path is found
        for grasp in grasp_control.keys():
            goal = grasp_control[grasp]
            print("Searching path to Move to object position")
            print("...Init=", init)
            print("...Goal=", goal)
            print("***Robot Control=", Robot_control)

            # Set the move query in Kautham
            kautham.kSetRobControlsNoQuery(node, Robot_control)
            kautham.kSetQuery(node, init, goal)

            # Solve query
            print("Solving Query to pick object")
            kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")  # Fresh GetPath
            path = kautham.kGetPath(node, 1)
            if path:
                print("-------- Path found: Moving to object position")
                print('Storing Grasp Controls Used = ', grasp)
                info.graspControlsUsed = grasp
                break
            else:
                print("**************************************************************************")
                print("Get path Failed! No Move possible, Infeasible Task Plan\nTrying next graspcontrol")
                print("**************************************************************************")

    # Write path to taskfile if found
    if path:
        print("STARTING TASKFILE WRITING")
        print(info.taskfile)
        info.taskfile.write("\t<Transit>\n")
        k = sorted(list(path.keys()))[-1][1] + 1  # number of joints
        p = sorted(list(path.keys()))[-1][0] + 1  # number of points in the path
        for i in range(p):
            tex = ''
            for j in range(k):
                tex = tex + str(path[i, j]) + " "
            ktmpb_python_interface.writePath(info.taskfile, tex)
        info.taskfile.write("\t</Transit>\n")
        kautham.kMoveRobot(node, goal)
    else:
        print("**************************************************************************")
        print("Get path Failed! No Move possible, Infeasible Task Plan")
        print("**************************************************************************")
        return False

    # Attach object (pick up the object)
    print("Picking object", obsName)
    kautham.kAttachObject(node, robotIndex, linkIndex, obsName)

    # DO NOT RETURN TO HOME OR INITIAL CONFIGURATION AFTER PICKING
    # Remove the code that tries to move back to init/home

    # The action ends here with the object now held by the specified hand.
    return True

def Pick_read(action_element):  # reading from the tamp configuration file
    for val in action_element.attrib:
        globals()[val] = action_element.attrib[val]

    pick = {}
    grasp = {}

    for el in action_element:
        try:
            globals()[el.tag] = int(el.text)
        except ValueError:
            try:
                globals()[el.tag] = [float(f) for f in str(el.text).strip().split()]
            except ValueError:
                globals()[el.tag] = str(el.text).strip()
        if el.tag == 'Graspcontrols':  # Variables with multiple entries
            grasp_name = el.get('grasp')
            graspcontrol = el.text
            graspcontrol = [float(f) for f in graspcontrol.split()]
            grasp[grasp_name] = graspcontrol
            globals()[el.tag] = grasp

        pick.update({el.tag: globals()[el.tag]})

    if len(grasp) == 0:
        print('No grasp conf found - This may be a problem')
    else:
        print('grasp = ', grasp)

    return pick

import kautham_python_interface as kautham
import ktmpb_python_interface
import xml.etree.ElementTree as ET
from collections import defaultdict

global path
global prepare_drink
prepare_drink = defaultdict(lambda: defaultdict(dict))

def PREPAREDRINK(node, prepare_drink, info, Line):
    print("**************************************************************************")
    print("  PREPARE DRINK ACTION  ")
    print("**************************************************************************")

    action = Line[0]
    rob = Line[1]
    bottle = Line[2]
    glass = Line[3]
    section = Line[4]

    print(action, rob, bottle, glass, section)

    try:
        arm_control = prepare_drink['ArmCont']
        hand_control = prepare_drink['HandCont']
        arm_init = prepare_drink['InitControls']
        arm_goal = prepare_drink['GoalControls']
        hand_region = prepare_drink['Regioncontrols']
        glass_position = prepare_drink['GlassPosition']
    except KeyError as e:
        print(f"Missing TAMP configuration tag: {e}")
        return False

    # TRANSIT: Move arm to bottle position
    print("TRANSIT: Moving arm to bottle position")
    kautham.kSetRobControlsNoQuery(node, arm_control)
    kautham.kSetQuery(node, arm_init, arm_goal)
    kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")
    path = kautham.kGetPath(node, 1)

    if not path:
        print("Failed to find a path to the bottle position")
        return False

    print("Path found: Moving to bottle position")
    info.taskfile.write("\t<Transit>\n")
    for i in range(len(path)):
        tex = " ".join(str(path[i, j]) for j in range(len(path[i])))
        ktmpb_python_interface.writePath(info.taskfile, tex)
    info.taskfile.write("\t</Transit>\n")
    kautham.kMoveRobot(node, arm_goal)

    # PICK: Grasp the bottle
    print("PICK: Grasping the bottle")
    kautham.kSetRobControlsNoQuery(node, hand_control)
    kautham.kSetQuery(node, hand_region, prepare_drink['Graspcontrols'])
    kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")
    path = kautham.kGetPath(node, 1)

    if not path:
        print("Failed to grasp the bottle")
        return False

    print("Path found: Grasping the bottle")
    info.taskfile.write("\t<Transit>\n")
    for i in range(len(path)):
        tex = " ".join(str(path[i, j]) for j in range(len(path[i])))
        ktmpb_python_interface.writePath(info.taskfile, tex)
    info.taskfile.write("\t</Transit>\n")
    kautham.kAttachObject(node, prepare_drink['Rob'], prepare_drink['Link'], bottle)

    # Start TRANSFER after picking the bottle
    print("Starting TRANSFER with bottle")
    info.graspedobject = True
    info.taskfile.write(f'\t<Transfer object = "{bottle}" robot = "{prepare_drink["Rob"]}" link = "{prepare_drink["Link"]}">\n')

    # TRANSFER: Move arm to glass position
    print("TRANSFER: Moving arm to glass position")
    kautham.kSetQuery(node, arm_goal, glass_position)
    kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")
    path = kautham.kGetPath(node, 1)

    if not path:
        print("Failed to move to the glass position")
        return False

    print("Path found: Moving to glass position")
    for i in range(len(path)):
        tex = " ".join(str(path[i, j]) for j in range(len(path[i])))
        ktmpb_python_interface.writePath(info.taskfile, tex)
    kautham.kMoveRobot(node, glass_position)

    # TRANSIT: Return arm to bottle position
    print("TRANSIT: Returning arm to bottle position")
    kautham.kSetQuery(node, glass_position, arm_goal)
    kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")
    path = kautham.kGetPath(node, 1)

    if not path:
        print("Failed to return to the bottle position")
        return False

    print("Path found: Returning to bottle position")
    for i in range(len(path)):
        tex = " ".join(str(path[i, j]) for j in range(len(path[i])))
        ktmpb_python_interface.writePath(info.taskfile, tex)
    kautham.kMoveRobot(node, arm_goal)

    # PLACE: Release the bottle
    print("PLACE: Releasing the bottle")
    kautham.kDetachObject(node, bottle)
    info.graspedobject = False
    info.taskfile.write("\t</Transfer>\n")

    # Return arm to initial position
    print("Returning arm to home position")
    kautham.kSetQuery(node, arm_goal, arm_init)
    kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")
    path = kautham.kGetPath(node, 1)

    if not path:
        print("Failed to return arm to home position")
        return False

    print("Path found: Returning arm to home position")
    info.taskfile.write("\t<Transit>\n")
    for i in range(len(path)):
        tex = " ".join(str(path[i, j]) for j in range(len(path[i])))
        ktmpb_python_interface.writePath(info.taskfile, tex)
    info.taskfile.write("\t</Transit>\n")
    kautham.kMoveRobot(node, arm_init)

    print("**************************************************************************")
    print("  PREPARE DRINK ACTION COMPLETED  ")
    print("**************************************************************************")
    return True

def Preparedrink_read(action_element):
    prepare_drink = {}

    for el in action_element:
        if el.tag in ['InitControls', 'GoalControls', 'Regioncontrols', 'Graspcontrols', 'GlassPosition']:
            prepare_drink[el.tag] = [float(x) for x in el.text.strip().split()]
        else:
            prepare_drink[el.tag] = el.text.strip()

    return prepare_drink

import kautham_python_interface as kautham
import ktmpb_python_interface
import xml.etree.ElementTree as ET
from collections import defaultdict

global prepare_drink
prepare_drink = defaultdict(lambda: defaultdict(dict))

def PREPARE_DRINK(node, prepare_drink, info, Line):
    print("**************************************************************************")
    print("  PREPARE DRINK ACTION  ")
    print("**************************************************************************")

    # Parse TAMP line: ['prepare_drink', 'MADAR', 'GLASS1', 'PREP-SECTION1']
    action = Line[0]
    robot = Line[1]
    glass = Line[2]
    section = Line[3]

    print(action, robot, glass, section)

    # Extract parameters from prepare_drink dictionary
    robotIndex = prepare_drink['Rob']
    objName = prepare_drink['Obj']  # The bottle name
    linkIndex = prepare_drink['Link']
    armControl = prepare_drink['ArmCont']
    handControl = prepare_drink['HandCont']
    initControls = prepare_drink['InitControls']
    goalControls = prepare_drink['GoalControls']
    regionControls = prepare_drink['Regioncontrols']
    graspControls = prepare_drink['Graspcontrols']
    glassPosition = prepare_drink['GlassPosition']

    ### Step 1: Move arm to bottle section ###
    print("Step 1: Moving arm to bottle section...")
    kautham.kSetRobControlsNoQuery(node, armControl)
    kautham.kSetQuery(node, initControls, goalControls)
    kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")
    path = kautham.kGetPath(node, 1)
    # Write path to taskfile with proper structure
    if path:
        print("-------- Path found: Moving robot to bottle section")
        info.taskfile.write("\t<Transit>\n")
        k = sorted(list(path.keys()))[-1][1] + 1
        p = sorted(list(path.keys()))[-1][0] + 1
        for i in range(p):
            tex = ''
            for j in range(k):
                tex += f"{path[i, j]} "
            ktmpb_python_interface.writePath(info.taskfile, tex)
        info.taskfile.write("\t</Transit>\n")
        kautham.kMoveRobot(node, goalControls)
    else:
        print("Get path failed! Unable to move arm to bottle section.")
        return False

    ### Step 2: Pick the bottle ###
    print("Step 2: Picking up the bottle...")
    kautham.kSetRobControlsNoQuery(node, handControl)
    kautham.kSetQuery(node, regionControls, graspControls)
    path = kautham.kGetPath(node, 1)
    if path:
        print("-------- Path found: Picking bottle")
        info.taskfile.write("\t<Transit>\n")
        k = sorted(list(path.keys()))[-1][1] + 1
        p = sorted(list(path.keys()))[-1][0] + 1
        for i in range(p):
            tex = ''
            for j in range(k):
                tex += f"{path[i, j]} "
            ktmpb_python_interface.writePath(info.taskfile, tex)
        info.taskfile.write("\t</Transit>\n")
        kautham.kAttachObject(node, robotIndex, linkIndex, objName)

        # Start Transfer
        info.graspedobject = True
        info.taskfile.write(f"\t<Transfer object=\"{objName}\" robot=\"{robotIndex}\" link=\"{linkIndex}\">\n")
    else:
        print("Get path failed! Unable to pick the bottle.")
        return False

    ### Step 3: Move arm to glass section ###
    print("Step 3: Moving arm to glass section...")
    kautham.kSetRobControlsNoQuery(node, armControl)
    kautham.kSetQuery(node, goalControls, glassPosition)
    path = kautham.kGetPath(node, 1)
    if path:
        print("-------- Path found: Moving robot to glass section")
        k = sorted(list(path.keys()))[-1][1] + 1
        p = sorted(list(path.keys()))[-1][0] + 1
        for i in range(p):
            tex = ''
            for j in range(k):
                tex += f"{path[i, j]} "
            ktmpb_python_interface.writePath(info.taskfile, tex)
        kautham.kMoveRobot(node, glassPosition)
    else:
        print("Get path failed! Unable to move to glass section.")
        return False

    ### Step 4: Return arm to bottle section ###
    print("Step 4: Returning arm to bottle section...")
    kautham.kSetRobControlsNoQuery(node, armControl)
    kautham.kSetQuery(node, glassPosition, goalControls)
    path = kautham.kGetPath(node, 1)
    if path:
        print("-------- Path found: Returning to bottle section")
        k = sorted(list(path.keys()))[-1][1] + 1
        p = sorted(list(path.keys()))[-1][0] + 1
        for i in range(p):
            tex = ''
            for j in range(k):
                tex += f"{path[i, j]} "
            ktmpb_python_interface.writePath(info.taskfile, tex)
        kautham.kMoveRobot(node, goalControls)
    else:
        print("Get path failed! Unable to return to bottle section.")
        return False

    ### Step 5: Place the bottle back ###
    print("Step 5: Placing the bottle back...")
    kautham.kSetRobControlsNoQuery(node, handControl)
    kautham.kSetQuery(node, graspControls, regionControls)
    path = kautham.kGetPath(node, 1)
    if path:
        print("-------- Path found: Placing the bottle")
        k = sorted(list(path.keys()))[-1][1] + 1
        p = sorted(list(path.keys()))[-1][0] + 1
        for i in range(p):
            tex = ''
            for j in range(k):
                tex += f"{path[i, j]} "
            ktmpb_python_interface.writePath(info.taskfile, tex)
        kautham.kDetachObject(node, objName)

        # End Transfer
        info.graspedobject = False
        info.taskfile.write("\t</Transfer>\n")
    else:
        print("Get path failed! Unable to place the bottle.")
        return False

    ### Step 6: Return arm to initial position ###
    print("Step 6: Returning arm to initial position...")
    kautham.kSetRobControlsNoQuery(node, armControl)
    kautham.kSetQuery(node, goalControls, initControls)
    path = kautham.kGetPath(node, 1)
    if path:
        print("-------- Path found: Returning arm to initial position")
        info.taskfile.write("\t<Transit>\n")
        k = sorted(list(path.keys()))[-1][1] + 1
        p = sorted(list(path.keys()))[-1][0] + 1
        for i in range(p):
            tex = ''
            for j in range(k):
                tex += f"{path[i, j]} "
            ktmpb_python_interface.writePath(info.taskfile, tex)
        info.taskfile.write("\t</Transit>\n")
        kautham.kMoveRobot(node, initControls)
    else:
        print("Get path failed! Unable to return to initial position.")
        return False

    print("**************************************************************************")
    print("  PREPARE DRINK ACTION COMPLETED SUCCESSFULLY")
    print("**************************************************************************")
    return True

def Prepare_drink_read(action_element):
    for val in action_element.attrib:
        globals()[val] = action_element.attrib[val]

    prepare_drink = {}
    for el in action_element:
        try:
            globals()[el.tag] = int(el.text)
        except ValueError:
            try:
                globals()[el.tag] = [float(f) for f in str(el.text).strip().split()]
            except ValueError:
                globals()[el.tag] = str(el.text).strip()
        prepare_drink.update({el.tag: globals()[el.tag]})

    return prepare_drink

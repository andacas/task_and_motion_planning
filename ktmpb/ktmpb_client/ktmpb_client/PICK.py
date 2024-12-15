import kautham_python_interface as kautham
import ktmpb_python_interface
import xml.etree.ElementTree as ET
from collections import defaultdict

global path
global pick
pick = defaultdict(lambda: defaultdict(dict))

def PICK(node, pick, info, Line):
    print("**************************************************************************")
    print("  PICK ACTION  ")
    print("**************************************************************************")

    # According to the domain: (pick ?r ?h ?o ?s)
    # Example line: ['pick','MADAR','LEFT-HAND','glass1','SHELF_3']
    action = Line[0]
    rob = Line[1]
    hand = Line[2]
    object_name = Line[3]
    section = Line[4]

    print(action, rob, hand, object_name, section)

    obsName = pick['Obj']
    robotIndex = pick['Rob']
    linkIndex = pick['Link']
    init = pick['Regioncontrols']
    Robot_control = pick['Cont']

    #Set robot control
    kautham.kSetRobControlsNoQuery(node, Robot_control)

    if 'Graspcontrols' in pick.keys():
        grasp_control = pick['Graspcontrols']
        for grasp in grasp_control.keys():
            goal = grasp_control[grasp]
            print("Searching path to Move to object position")
            print("...Init=", init)
            print("...Goal=", goal)
            print("***Robot Control=", Robot_control)

            kautham.kSetRobControlsNoQuery(node, Robot_control)
            kautham.kSetQuery(node, init, goal)

            print("Solving Query to pick object")
            kautham.kSetPlannerParameter(node, "_Incremental (0/1)", "0")
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

    if path:
        print("STARTING TASKFILE WRITING")
        print(info.taskfile)
        info.taskfile.write("\t<Transit>\n")
        k = sorted(list(path.keys()))[-1][1]+1 #number of joints
        p = sorted(list(path.keys()))[-1][0]+1 #number of points in the path
        for i in range(p):
            tex=''
            for j in range(0,k):
                tex=tex + str(path[i,j]) + " "
            ktmpb_python_interface.writePath(info.taskfile,tex)
        info.taskfile.write("\t</Transit>\n")
        kautham.kMoveRobot(node, goal)
    else:
        print("**************************************************************************")
        print("Get path Failed! No Move possible, Infeasible Task Plan")
        print("**************************************************************************")
        return False#break



    # Attach object (pick up the object)
    print("Picking object", obsName)
    kautham.kAttachObject(node, robotIndex, linkIndex, obsName)

    # Start the Transfer section now that we have the object
    # info.graspedobject = True means we are now in a transfer
    info.graspedobject = True
    info.taskfile.write("\t<Transfer object = \"%s\" robot = \"%d\" link = \"%d\">\n" % (obsName, robotIndex, linkIndex))

    # DO NOT return to home configuration; no further path writing here
    # The transfer will continue until the object is released in another action.

    return True

def Pick_read(action_element): #reading from the tamp configuration file

    for val in action_element.attrib:
        globals()[val] = action_element.attrib[val]

    pick = {}
    grasp={}

    for el in action_element:
        try:
            globals()[el.tag] = int(el.text)
        except:
            try:
                globals()[el.tag] = [float(f) for f in str(el.text).strip().split()]
            except:
                globals()[el.tag] = str(el.text).strip()
        if el.tag == 'Graspcontrols': #variables with multiple entries should be added the same way
            grasp_name = el.get('grasp')
            graspcontrol= el.text
            graspcontrol=[float(f) for f in graspcontrol.split()]
            grasp[grasp_name]= graspcontrol
            globals()[el.tag] = grasp

        pick.update({el.tag : globals()[el.tag]})

    if len(grasp)==0:
        print('No grasp conf found - This may be a problem')
    else:
        print('grasp = ', grasp)

    return pick

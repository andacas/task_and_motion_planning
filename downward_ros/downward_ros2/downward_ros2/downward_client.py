#!/usr/bin/env python3

from __future__ import print_function

import sys
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from downward_interfaces.srv import Plan
import pathlib

class DownwardClient(Node):
    def __init__(self):
        super().__init__('downward_client')
        self.downward_client = self.create_client(Plan, 'downward_service')
        while not self.downward_client.wait_for_service(timeout_sec=1.0):
            print("Service not available, waiting again...")

        self.req = Plan.Request()

        # problem_param:
        self.declare_parameter('problem_param', 'block_world')

        # domain_param:
        self.declare_parameter('domain_param', 'block_world')

    def send_request(self, problem, domain, evaluator, search):
        self.req.problem = problem
        self.req.domain = domain
        self.req.evaluator = evaluator
        self.req.search = search
        self.future = self.downward_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    my_downward_client = DownwardClient()

    problem_param = my_downward_client.get_parameter("problem_param").value
    domain_param = my_downward_client.get_parameter("domain_param").value

    downward_ros_path = get_package_share_directory("downward_interfaces")
    problemFile = downward_ros_path + "/pddl/" + problem_param
    domainFile = downward_ros_path + "/pddl/" + domain_param + ".pddl"
    print("Requesting FF plan for problem file %s and domain file %s"%(problemFile, domainFile))

    strProblemFile=open(problemFile,"r").read()
    print("---------- ProblemFile ------------")
    print(strProblemFile)

    strDomainFile=open(domainFile,"r").read()
    print("---------- DomainFile -------------")
    print(strDomainFile)

    # 1. Default Fast Downward with FF heuristic (emulating FF planner)
    evaluator = ""  # Defaults to "hff=ff()"
    search = ""     # Defaults to "lazy_greedy([hff], preferred=[hff])"
    print("------ CALLING fast-downward as FF -------")
    r = my_downward_client.send_request(strProblemFile, strDomainFile, evaluator, search)
    if r.response:
        print("Solution plan is: ", r.plan)
    else:
        print(r.plan)

    # 2. Context-Enhanced Additive Heuristic with Lazy Greedy Search
    print("------ CALLING fast-downward using context-enhanced additive heuristic -------")
    evaluator = "\"hcea=cea()\""
    search = "\"lazy_greedy([hcea], preferred=[hcea])\""
    r = my_downward_client.send_request(strProblemFile, strDomainFile, evaluator, search)
    if r.response:
        print("Solution plan is: ", r.plan)
    else:
        print(r.plan)

    # 3. Lazy Weighted A with Context-Enhanced Additive Heuristic (CEA) and Preferred Operators*
    print("------ CALLING Eager Greedy Best-First Search with Preferred Operators -------")
    evaluator = "\"hff=ff()\""  # FF heuristic for efficient planning
    search = "\"eager_greedy([hff], preferred=[hff], boost=100)\""
    r = my_downward_client.send_request(strProblemFile, strDomainFile, evaluator, search)
    if r.response:
        print("Solution plan is: ", r.plan)
    else:
        print(r.plan)

    # 4. Lazy Weighted A with Context-Enhanced Additive Heuristic (CEA) and Preferred Operators*
    print("------ CALLING Lazy Weighted A Search with Preferred Operators -------")
    evaluator = "\"hcea=cea()\""  # context-enhanced additive heuristic for broader contextual awareness
    search = "\"lazy_wastar([hcea], preferred=[hcea], w=2, reopen_closed=true)\""
    r = my_downward_client.send_request(strProblemFile, strDomainFile, evaluator, search)
    if r.response:
        print("Solution plan is: ", r.plan)
    else:
        print(r.plan)



    # 5. Lazy Weighted A with Context-Enhanced Additive Heuristic (CEA) and Preferred Operators*
    print("------ CALLING the landmark heuristic combined with goal-counting within a lazy weighted A* search strategy. -------")
    evaluator = "\"goalcount=goalcount()\""
    search = "\"lazy_greedy([goalcount], preferred=[goalcount])\""

    r = my_downward_client.send_request(strProblemFile, strDomainFile, evaluator, search)
    if r.response:
        print("Solution plan is: ", r.plan)
    else:
        print(r.plan)
        
    # 5. Lazy Weighted A with Context-Enhanced Additive Heuristic (CEA) and Preferred Operators*
    print("------ CALLING the landmark heuristic combined with goal-counting within a lazy weighted A* search strategy. -------")
    evaluator = "\"lmcut=lmcut()\""
    search = "\"lazy_wastar([lmcut], w=2, reopen_closed=true)\""
    r = my_downward_client.send_request(strProblemFile, strDomainFile, evaluator, search)
    if r.response:
        print("Solution plan is: ", r.plan)
    else:
        print(r.plan)
    my_downward_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

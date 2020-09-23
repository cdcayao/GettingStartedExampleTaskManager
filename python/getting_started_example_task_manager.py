#!/usr/bin/env python3
import pwd, os, sys
import time, random
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor

from lib.PythonCommander import PythonCommander
from lib.PythonCommanderHelper import PythonCommanderHelper
import lib.CommonOperations as cmn_ops 

def LaunchMoveToHub(cmdr,workstate,hub,speed,project):
    # Execute this in a thread
    res,seq = cmdr.MoveToHub(workstate,hub,speed,project_name=project)
    code = cmdr.WaitForMove(seq)
    return code

def LaunchMoveToPose(cmdr,workstate, pose, tol, complete_move, complete_move_type,speed,project):
    # Execute this in a thread
    res,seq = cmdr.MoveToPose(workstate,pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], tol[0], tol[1], tol[2], tol[3], tol[4], tol[5], complete_move, complete_move_type, speed,project_name=project)
    code = cmdr.WaitForMove(seq)
    return code

class TaskPlanner():
    replan_attempts = 1
    timeout = 0.5

    def __init__(self,ip,fp):
        # Setup the PythonCommander which is responsible for sending commands to the 
        # RTR Controller that control the robot/s or simulation
        self.init_logging(fp)
        self.log(f'Replan Attempts: {self.replan_attempts}')
        self.log(f'Move Timeout: {self.timeout}')
        
        self.ip_addr = ip
        self.cmdr = PythonCommander(self.ip_addr, 9999) #TODO: pass IP as argument.
        self.cmdr.Reconnect()

        code, data = self.cmdr.GetMode()
        if data == 'FAULT':
            code = self.cmdr.ClearFaults()
            if code != self.cmdr.SUCCESS:
                self.log('Failed to clear faults!')
                return
        
        # Commander helper that communicates with the controller using a REST api
        self.helper = PythonCommanderHelper(self.ip_addr)

        # Request the group info from the control panel
        # This assumes there is only one group on the control panel!
        self.group_info = self.helper.get_group_info()
        self.group = None
        for group_name,info in self.group_info.items():

            self.group = group_name
            self.project_names = info['projects']        

        # Nested dictionary that contains all projects information of the form:
        # {project name: {workstates: [str], hubs: [str]}
        self.project_info = self.helper.get_project_info(self.group_info[self.group]['projects'])
        # self.project_names = self.group_info[self.group]['projects']

        # Call startup sequence which calls InitGroup for each project and then BeginOperationMode
        self.log('Startup sequence...')
        cmn_ops.startup_sequence(self.cmdr,self.project_info,self.group)

        # Put each robot on the roadmap
        self.log('Putting robots on roadmap...')
        cmn_ops.put_on_roadmap(self.cmdr,self.project_info,self.group,hub='staging')

        # Set interupt behavior
        for project_idx in range(0,len(self.project_info)):
            name = self.project_names[project_idx]
            self.cmdr.SetInterruptBehavior(self.replan_attempts,self.timeout,project_name=name)

    def AcquireTargets(self, cmdr, project_idx):
        #x1 = random.uniform(-0.1, -0.5) #
        x1 = random.uniform(-0.4, -0.6) #
        #x2 = random.uniform(-0.5, -0.9) #         
        x2 = random.uniform(-0.4, -0.6) #

        y = random.uniform(0.4, 0.6)

        pose = []
        
        if project_idx == 0:
            pose = [x1, y, 0.05, 0.0,3.14,0.0]
        elif project_idx == 1:
            pose = [x1, y, 0.05, 0.0,3.14,0.0]

        return pose

    def LaunchPickAndPlace(self, cmdr,workstate, hub, pose, tol, complete_move, complete_move_type,speed,project):
        # Execute this in a thread
        pick_code = 1
        pick_code_2 = 1
        place_code = 1

        group = 'Test'
        group_info = self.helper.get_group_info()
        project_info = self.helper.get_project_info(group_info[group]['projects'])
        project_names = group_info[group]['projects']

        res,seq = cmdr.MoveToPose(workstate,pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], tol[0], tol[1], tol[2], tol[3], tol[4], tol[5], complete_move, complete_move_type, speed,project_name=project)
        pick_code = cmdr.WaitForMove(seq)
        if pick_code != 0:
            pick_code = 1
        else:
            pick_code = 0
        
        if pick_code == 0:
            for project_idx in range(0,len(self.project_info)):
                if project == project_names[project_idx]:
                    self.retreat_idx = project_idx

            res, seq = cmdr.BlindMove(workstate, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], 0, speed, False, project_name=project)
            pick_code_2 = cmdr.WaitForMove(seq) 
            if pick_code_2 != 0:
                pick_code_2 = 1
            else:
                pick_code_2 = 0

            if pick_code_2 == 0:
                for project_idx in range(0,len(self.project_info)):
                    if project == project_names[project_idx]:
                        self.retreat_idx = project_idx

                res,seq = cmdr.MoveToHub(workstate,hub,speed,project_name=project)
                place_code = cmdr.WaitForMove(seq)
                if place_code != 0:
                    place_code = 1
                else:
                    place_code = 0

        return pick_code + pick_code_2 + place_code

    def init_logging(self,fp):
        self.fp = fp
        self.fp.write('\n')

    def log(self,msg):
        log_msg = f'[{datetime.now()}] {msg}\n'
        self.fp.write(log_msg)
        print(log_msg)
    
    def advance_hub(self,project_idx):
        project = self.project_names[project_idx]
        # workstate = self.project_info[project]['workstates'][0]
        workstate = 'no_part'
        hub_list = self.hubs[project_idx]
        hub_idx = self.hub_idxs[project_idx]

        if hub_idx > len(hub_list)-1:
            self.end_times[project_idx] = time.time()
            self.pick_and_place[project_idx] = False
            self.threads[project_idx] = None
            self.log(f'Project {self.project_names[project_idx]} has finished!')
        else:
            hub = hub_list[hub_idx]
            future = self.executor.submit(LaunchMoveToHub,self.cmdr,workstate,hub,self.speed,project)
            self.threads[project_idx] = future
            # print(f'Advancing project {self.project_names[project_idx]} to hub {hub}!')

    def pick_part(self,project_idx, pose, tol):
        project = self.project_names[project_idx]
        # workstate = self.project_info[project]['workstates'][0]
        workstate = 'no_part'
        hub_list = self.hubs[project_idx]
        hub_idx = self.hub_idxs[project_idx]

        if hub_idx > len(hub_list)-1:
            self.end_times[project_idx] = time.time()
            self.pick_and_place[project_idx] = False
            self.threads[project_idx] = None
            self.log(f'Project {self.project_names[project_idx]} has finished!')
        else:
            hub = hub_list[hub_idx]
            future = self.executor.submit(LaunchMoveToPose,self.cmdr,workstate,pose,tol,self.complete_move,self.complete_move_type, self.speed,project)
            self.threads[project_idx] = future
            # print(f'Advancing project {self.project_names[project_idx]} to hub {hub}!')

    def pick_and_place_part(self,project_idx, pose, tol):
        project = self.project_names[project_idx]
        # workstate = self.project_info[project]['workstates'][0]
        workstate = 'no_part'
        hub_list = self.hubs[project_idx]
        hub_idx = self.hub_idxs[project_idx]

        if hub_idx > len(hub_list)-1:
            self.end_times[project_idx] = time.time()
            self.pick_and_place[project_idx] = False
            self.threads[project_idx] = None
            self.log(f'Project {self.project_names[project_idx]} has finished!')
        else:
            hub = hub_list[hub_idx]
            future = self.executor.submit(self.LaunchPickAndPlace,self.cmdr,workstate,hub,pose,tol,self.complete_move,self.complete_move_type, self.speed,project)
            self.threads[project_idx] = future
            # print(f'Advancing project {self.project_names[project_idx]} to hub {hub}!')

    def retract_to_staging(self,project_idx):
        self.log(f'Retracting project {self.project_names[project_idx]} to staging!')
        project = self.project_names[project_idx]
        # workstate = self.project_info[project]['workstates'][0]
        workstate = 'no_part'
        hub = 'staging'
        future = self.executor.submit(LaunchMoveToHub,self.cmdr,workstate,hub,self.speed,project)
        self.threads[project_idx] = future

    def start(self):
        '''
        Run the hub cycle which is a list of hubs for each robot
        '''
        self.speed = 1.0
        self.complete_move = 0
        self.complete_move_type = 0

        hub_1 = ['place_1_1', 'place_1_2', 'place_1_3','place_1_4','place_2_1', 'place_2_2', 'place_2_3','place_2_4']
        hub_2 = ['place_1_1', 'place_1_2', 'place_1_3','place_1_4','place_2_1', 'place_2_2', 'place_2_3','place_2_4']
        self.hubs = []
        
        # Loop through both projects hub lists and match the hub lists to the correct robot
        # This will also check that all hub hubs typed exist, and will prevent 4004 errors
        for project_idx in range(0,len(self.project_info)):
            project_name = self.project_names[project_idx]
            hub_list = self.project_info[project_name]['hubs']
            skip = False
            for hub in hub_1:
                if hub not in hub_list:
                    self.log(f'hub ({hub}) doesnt exist in project ({project_name})')
                    skip = True
                    break
            if not skip:
                self.hubs.append(hub_1)
                self.retreat_idx = project_idx
            else:
                skip = False
                for hub in hub_2:
                    if hub not in hub_list:
                        self.log(f'hub ({hub}) doesnt exist in project ({project_name})')
                        skip = True
                        break
                if not skip:
                    self.hubs.append(hub_2)
                    self.retreat_idx = project_idx
                    
        assert(len(self.hubs)==2),'Failed to assign the hub lists to the loaded projects!'

        self.executor = ThreadPoolExecutor(max_workers=4)
        self.threads = [None]*len(self.project_info)
        self.pick_and_place = [True]*len(self.project_info) # status of the bots. False when back at home
        self.end_times = [None]*len(self.project_info) # cycle time stop watch
        self.hub_idxs = [0]*len(self.project_info) # current index of the hub list
        self.interlocking = [False]*len(self.project_info) # project idx of the robot that had to retreat

        self.log(f'Beginning hub cycle...')
        self.start_time = time.time()

        project_list = list(range(0,len(self.project_info)))
        project_list.reverse()

        pose = self.AcquireTargets(self.cmdr, project_idx)

        tol = [0.1,0.1,0.1,3.14,3.14,3.14]

        self.pick_and_place_part(1, self.AcquireTargets(self.cmdr, 1), tol)
        self.pick_and_place_part(0, self.AcquireTargets(self.cmdr, 0), tol)
        
        # While both projects haven't finished their cycles
        while True in self.pick_and_place:

            for project_idx in range(0,len(self.project_info)):

                pose = self.AcquireTargets(self.cmdr, project_idx)
                if self.pick_and_place[project_idx]: # If the project isn't finished
                    res = self.threads[project_idx].done() # Check if the PickAndPlace call thread terminated
                    if res:
                        code = self.threads[project_idx].result() # Catch the move result and determine what to do
                        
                        if code == self.cmdr.SUCCESS:
                            if not self.interlocking[project_idx]:
                                # If your previous move was not a retraction, that means you completed a pick and place move
                                # and need to increase the hub idx
                                self.log(f'Project {self.project_names[project_idx]} completed move to {self.hubs[project_idx][self.hub_idxs[project_idx]]}!')
                                self.hub_idxs[project_idx] += 1 
                            self.pick_and_place_part(project_idx, pose, tol)
                            self.interlocking[project_idx] = False
                        elif self.retreat_idx == project_idx:
                            self.retract_to_staging(project_idx)
                            self.interlocking[project_idx] = True
                        else:
                            self.pick_and_place_part(project_idx, pose, tol)
                            self.interlocking[project_idx] = False

        self.retract_to_staging(1)
        self.retract_to_staging(0)        

        for project_idx in range(0,len(self.project_info)):
            hub_time_str = f'{self.project_names[project_idx]} hub cycle took: {self.end_times[project_idx]-self.start_time}'
            self.log(hub_time_str)        

def main():
    # If ip address is passed, use it
    if len(sys.argv) != 1:
        ip_addr = str(sys.argv[1])
    else: # Default IP address
        ip_addr = "127.0.0.1"
    print(f'Setting ip address of Realtime Controller to: {ip_addr}')
    
    fp = open('hub_log.txt','a')
    try:
        task_planner = TaskPlanner(ip_addr,fp)
        task_planner.start()
    finally:
        fp.close()
    
if __name__=="__main__":
    main()
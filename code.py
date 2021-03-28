from collections import defaultdict, deque
from copy import deepcopy

######### INPUT TAKING ########
print("START")
V = int(input("Enter the number of vertices: "))
adj = [[0 for i in range(V)] for j in range(V)]
# print("Now start entering the edge distances")
for i in range(V):
    for j in range(V):
        inp = input(f"e{i+1}{j+1}-> ")
        if inp=="INF" or inp=="inf":
            adj[i][j]=float("inf")
        else:
            adj[i][j] = float(inp)

src={}
dest={}
battery_status={}
charging_rate={}
discharging_rate={}
Max_battery={}
avg_speed={}
K = int(input("Number of Cars: "))

for i in range(K):
    src[i]=int(input(f"source node of {i+1}th car: "))-1
    dest[i]=int(input(f"destination node of {i+1}th car: "))-1
    battery_status[i]=float(input(f"initial battery status of {i+1}th car: "))
    charging_rate[i]=float(input(f"charging rate of {i+1}th car: "))
    discharging_rate[i]=float(input(f"discharging rate of {i+1}th car: "))
    Max_battery[i]=float(input(f"Max battery capacity of {i+1}th car: "))
    avg_speed[i]=float(input(f"average speed of {i+1}th car: "))

arrival_time = [[float("inf") for i in range(K)] for i in range(V)]

################ INPUT TAKING DONE ###############

################ CLASS DEFINITION ################

class car:
    def __init__(self,id, src, dest, battery_status, charging_rate, discharging_rate, Max_battery, avg_speed,adj):
        '''
        Initialize object with: car ID, source node, destination node, 
        initial battery status, charging and discharging rates, 
        battery capacity, average speed, adjacency matrix
        '''
        self.id = id
        self.src = src
        self.dest = dest
        self.battery_status = battery_status
        self.charging_rate = charging_rate
        self.discharging_rate=discharging_rate
        self.Max_battery=Max_battery
        self.avg_speed=avg_speed
        self.path=[]
        self.graph = deepcopy(adj)
        for i in range(adj.shape[0]):
            for j in range(adj.shape[0]):
                if adj[i][j]!=float("inf"):
                    if (adj[i][j]/self.discharging_rate)>self.Max_battery:
                        self.graph[i][j]=float("inf")

        self.edges=[]
        for i in range(adj.shape[0]):
            temp=[]
            for j in range(adj.shape[0]):
                if self.graph[i][j]!=float('inf'):
                    temp.append(j)
            self.edges.append(temp)

    def shortest_path(self):
        """
        IN : NA
        OUT: NA
        
        1. Calculate, store the min. distances of nodes from source. Also store parent of each node.
        2. Battery management for each car.
        (Assuming no collisions, to be considered later)
        """
        ##### Dijkstra's algorithm for getting the shortest path between the src and dest node ####
        initial = self.src
        shortest_paths = defaultdict(lambda:(None, float("inf"))) # dictionary for storing the parent and distance from the src node of a node
        shortest_paths[initial] = (None, 0)
        visited = defaultdict(bool)
        q = PriorityQueue()
        q.put((0,initial))
        while(not q.empty()):
            curr_node = q.get()[1]
            if(visited[curr_node]):
                continue
            visited[curr_node] = True
            for next_node in self.edges[curr_node]:
                if(shortest_paths[curr_node][1]+self.graph[curr_node][next_node]<shortest_paths[next_node][1]):
                    shortest_paths[next_node] = (curr_node, shortest_paths[curr_node][1]+graph[curr_node][next_node])
                    q.put((shortest_paths[next_node][1], next_node))
        
        #### Dijkstra's Algorithm End ####
        # Tarcing the path from the dest to src node using the shortest path dictionary
        current_node = self.dest
        while current_node is not None:
            self.path.append(current_node)
            next_node = shortest_paths[current_node][0]
            current_node = next_node
        # self.path contains the path from dest to src
        # so for getting the path from src to dest, list reversal is done
        self.path = self.path[::-1]

        self.distance = defaultdict(int)  # for storing the distance of every node in the obtained path from dest node 
        for i in range(len(self.path)-2,-1,-1):
            self.distance[self.path[i]] = self.graph[self.path[i]][self.path[i+1]] + self.distance[self.path[i+1]]


        self.min_time = defaultdict(int) # for storing the minimum time required to reach dest node from the current node
        temp_battery_status = self.battery_status
        self.charge_time_required = defaultdict(int)
        for i in range(len(self.path)):
            if(i!=0):
                temp_battery_status -= self.graph[self.path[i-1]][self.path[i]]/self.discharging_rate
            self.min_time[self.path[i]] = self.distance[self.path[i]]/self.avg_speed
            min_charge_required = self.distance[self.path[i]]/self.discharging_rate
            self.charge_time_required[self.path[i]] = 0
            if(temp_battery_status<=min(self.Max_battery, min_charge_required)):
                self.charge_time_required[self.path[i]] = (min(self.Max_battery, min_charge_required)-temp_battery_status)/self.charging_rate
                self.min_time[self.path[i]] += self.charge_time_required[self.path[i]]

            arrival_time[self.path[i]][self.id] = 0
            if(i!=0):
                arrival_time[self.path[i]][self.id] = arrival_time[self.path[i-1]][self.id]+self.min_time[self.path[i-1]]-self.min_time[self.path[i]]

        self.total_time_taken = self.min_time[self.src]


    def update_parameters(self, node, waiting_time):
        """
        IN : current node, waiting time at current node
        OUT: NA
        
        Wait time management, given there is one charger at each node.
        """
        # with the given waiting time at a node, the arrival time and min_time for all the upcoming nodes in the
        # path will be increased by that waiting time amount
        self.total_time_taken += waiting_time
        self.min_time[node] += waiting_time
        update = False
        for i in range(len(self.path)):
            if(update):
                self.min_time[self.path[i]] += waiting_time
                arrival_time[self.path[i]][self.id] += waiting_time
            if(self.path[i]==node):
                update=True

################ CLASS DEFINITION END ################

# calculate the minimum time required, stored as an array for every car, from a node to its destination 
# our initial assumption is no car competes for charging
all_cars = {}
for i in range(K):
    obj = car(i,src[i],dest[i],battery_status[i],charging_rate[i],discharging_rate[i],Max_battery[i],avg_speed[i],adj)
    obj.shortest_path()
    all_cars[i] = obj

# the code bellow updates the minimum time required by resolving competition occuring at nodes based on the initial assumption
# we define an event as a car arriving or departing at a node
# we make a list of all such events, sorted in ascending order, exceute them one by one, and resolve any competition 
def schedule(cars_list, prev_charging_car, charge_time_left_list, conflict_node):
    """
    Decides which car to charge from in case of competition

    IN : 
        cars_list             - list of cars waiting
        prev_charging_car     - car charging currently
        charge_time_left_list - V-dimensional vector
        conflict_node         - address of conflict node
    OUT: 
        ID of car to be charged
    """
    car_obj_list = [w[0] for w in cars_list]
    if len(car_obj_list)==1:
        return car_obj_list[0].id
    temp = sorted([sorted([(i, w.min_time[conflict_node] + charge_time_left_list[x.id]) for j,w in enumerate(car_obj_list) if j!=i], key=lambda y:-y[1]) for i,x in enumerate(car_obj_list)], key=lambda y:y[0][1])
    return temp[0][0][0]
   
# make global time list
global_time_list = []

# global time list stores a list of events. 
# each item is a list with a car object, string signifying arival or departure, node at which event occurs, time of event
for conflict_node in range(V):
    cars_present = [all_cars[i] for i in range(K) if arrival_time[conflict_node][i]!=float("inf")]
    charge_time_left_list = [0]*K
    for i in cars_present:
        arr_time = arrival_time[conflict_node][i.id]
        dep_time = arr_time + i.charge_time_required[conflict_node]
        charge_time_left_list[i.id] = int(i.charge_time_required[conflict_node])
        global_time_list.append([i,"arrival", conflict_node, arr_time])
        global_time_list.append([i,"departure", conflict_node, dep_time])
global_time_list.sort(key=lambda x: x[3])

cars_list_list = [[]]*V  # (num nodes X num cars at node) list of cars present at a node
prev_event_time_list = [0]*V  # list of time of the event before the current one for each node
prev_charging_cars_list = [-1]*V  # list of car charging at each node before the current event occurs

# resolve all conflicts in global_time_list starting at index 0
while(len(global_time_list)!=0):  # continue till all events are processed
    event = global_time_list.pop(0)  # get the soonest occuring event(current event)
    conflict_node = event[2]  # the node at which current event

    if(event[1]=="departure"):  # if the event is a departure
        poped=False
        index_event_id = -1
        for i in range(len(cars_list_list[conflict_node])):  
            if(cars_list_list[conflict_node][i][0].id==event[0].id):
                index_event_id = i  # get the index of the car of the current event in the list of cars of the node of the current event
        waiting_time = event[3]- prev_event_time_list[conflict_node]  # the cars not charging, if any, have waited from the previous event till now
        prev_event_time_list[conflict_node] = int(event[3])
        charge_time_left_list[prev_charging_cars_list[conflict_node]] -= waiting_time  # the car carging from the previous event has charged till now 
        if(prev_charging_cars_list[conflict_node]==event[0].id):  # if the car departing is the car that was charging then remove it from the list of cars at that node
            cars_list_list[conflict_node].pop(index_event_id)     # the opposite can happen when a new car wins the competition forcing the older car to wait till its fully charged 
            poped=True
        
        # arival times are calculated according to our initial assumption
        # so if a car waits at a node, that time is added to the arrival times to the future nodes in its path
        if(len(cars_list_list[conflict_node])>0):  # if there are cars at the node, update the arival times of cars waiting, update the arrival times
                                                   # , in array and gobal list, for the waiting cars 
            for i in range(len(cars_list_list[conflict_node])):  
                if(cars_list_list[conflict_node][i][0].id!=prev_charging_cars_list[conflict_node]):
                    index_list_in_global_time_list=[]
                    for idx,listn in enumerate(global_time_list):
                        if listn[0].id==cars_list_list[conflict_node][i][0].id and listn[2]==conflict_node:
                            index_list_in_global_time_list.append(idx)

                    cars_list_list[conflict_node][i][0].update_parameters(conflict_node, waiting_time)  # update arrival times
                    if index_list_in_global_time_list:
                        for index_in_global_time_list in index_list_in_global_time_list:
                            old_list = global_time_list[index_in_global_time_list]
                            arr_time = arrival_time[conflict_node][cars_list_list[conflict_node][i][0].id]
                            dep_time = arr_time + cars_list_list[conflict_node][i][0].charge_time_required[conflict_node]
                            new_list = [cars_list_list[conflict_node][i][0],"arrival", old_list[2], arr_time] if old_list[1]=="arrival" \
                                        else [cars_list_list[conflict_node][i][0],"departure", old_list[2], dep_time]
                            global_time_list[index_in_global_time_list] = new_list
                    global_time_list.sort(key=lambda x: x[3])

        if poped and len(cars_list_list[conflict_node])>0:  # if the cars at the node have changed and there are cars at the node, resolve competition to determine which car charges next 
            prev_charging_cars_list[conflict_node] = schedule(cars_list_list[conflict_node], prev_charging_cars_list[conflict_node], charge_time_left_list, conflict_node)

    else:  # if arrival event, basically same as departure, with car being added to car_list 
        index_event_id = -1
        for i in range(len(cars_list_list[conflict_node])):
            if(cars_list_list[conflict_node][i][0].id==event[0].id):
                index_event_id = i
        waiting_time = event[3] - prev_event_time_list[conflict_node]
        prev_event_time_list[conflict_node] = event[3]
        cars_list_list[conflict_node].append(event)
        charge_time_left_list[prev_charging_cars_list[conflict_node]] -= waiting_time
        if(len(cars_list_list[conflict_node])>0):   # changing departure time
            for i in range(len(cars_list_list[conflict_node])):
                if(cars_list_list[conflict_node][i][0].id!=prev_charging_cars_list[conflict_node]):
                    index_list_in_global_time_list=[]
                    for idx,listn in enumerate(global_time_list):
                        if listn[0].id==cars_list_list[conflict_node][i][0].id and listn[2]==conflict_node:
                            index_list_in_globalp_time_list.append(idx)

                    cars_list_list[conflict_node][i][0].update_parameters(conflict_node, waiting_time)
                    if index_list_in_global_time_list:
                        for index_in_global_time_list in index_list_in_global_time_list:
                            old_list = global_time_list[index_in_global_time_list]
                            arr_time = arrival_time[conflict_node][cars_list_list[conflict_node][i][0].id]
                            dep_time = arr_time + cars_list_list[conflict_node][i][0].charge_time_required[conflict_node]
                            new_list = [cars_list_list[conflict_node][i][0],"arrival", old_list[2], arr_time] if old_list[1]=="arrival" \
                                        else [cars_list_list[conflict_node][i][0],"departure", old_list[2], dep_time]
                            global_time_list[index_in_global_time_list] = new_list
                    global_time_list.sort(key=lambda x: x[3])

        if len(cars_list_list[conflict_node])>0:
            prev_charging_cars_list[conflict_node] = schedule(cars_list_list[conflict_node], prev_charging_cars_list[conflict_node], charge_time_left_list, conflict_node)  # scheduling

############ RESULT ############
'''
The routes can be obtained using the 2-D array arrival_time which conatins 
all the arrival time of a vehicle at a node. The total time taken by a vehicle 
can be obatined by its total_time_taken attribute.
'''
# Printing the routes of every vehicle
for i in range(len(all_cars)):
    print("ROUTE OF VEHICLE {}".format(i+1))
    obj = all_cars[i]
    print(f"CITY  ARRIVAL TIME")
    for j in range(len(obj.path)):
        curr_node = obj.path[i]
        time_arr = arrival_time[curr_node][obj.id]
        print("{}  {}".format(curr_node+1, time_arr))
    print()

# Printing the overall max(Tr)

all_Tr = [all_cars[i].total_time_taken for i in range(len(all_cars))]
print("The minimised maximum time taken is equal to {}".format(max(all_Tr)))
print("END")
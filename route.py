import copy
import math
import pickle
from functools import total_ordering
import detour

# from types import List, Dict, Tuple, Node

from typing import List, Dict, Tuple
Node = Tuple[int, int]
Time = float



class PathData:

    def __init__(self, filepath : str): #Pathdata Classを呼んだときに行われる処理
        with open(filepath, "rb") as f:
            mapper = pickle.load(f) #距離のデータが入っているファイルを参照

        self.nodes = mapper.default_targets
        self.home_poses = mapper.starting_point
        self.distance_of = {node_pair:path[1] for node_pair, path in mapper.paths.items()}

        self.nodes_to_index:Dict[Node, int] = {}
        for i, node in enumerate(self.nodes):
            self.nodes_to_index[node] = i + 1


    # Calc distance of two points、２チェックポイント間の距離を計算
    def distance(self, c1:Node, c2:Node) -> float:
        if c1 == c2: return 0.0
        return self.distance_of[(c1, c2)] #__init__で定義


    # Calc distance of coords
    def distance_of_nodes(self, nodes:List[Node]) -> float:

        distance = 0.0
        for i in range(len(nodes)-1):
            distance += self.distance(nodes[i], nodes[i+1])

        return distance


class DroneProperty:

    def __init__(self, pathdata:PathData):
        self.pathdata = pathdata
        self.home_pos = pathdata.home_poses[0]
        self.speed = 0.5
        self.battery_capacity = 3000.0
        self.battery_per_distance = 1.0



class Drone:

    def __init__(self, props:DroneProperty):
        self.props = props

        self.pos_history = [self.props.home_pos]
        self.last_pos = self.props.home_pos
        self.total_distance = 0.0
        self.elapsed_time = 0.0
        self.battery_remain = self.props.battery_capacity


    def _distance(self, c1:Node, c2:Node) -> float: #２チェックポイント間の距離
        return self.props.pathdata.distance(c1, c2)


    def move_to(self, target:Node) -> None: #動けるかチェックはせずに、チェックポイントにドローンを動かす
        if self.last_pos == target: return

        c_distance = self._distance(self.last_pos, target)

        self.pos_history.append(target)
        self.last_pos = target

        self.total_distance += c_distance
        self.elapsed_time += c_distance * self.props.speed

        self.battery_remain -= c_distance * self.props.battery_per_distance
        if self.battery_remain < 0:
            raise RuntimeError('Out of battery.')

        if self.last_pos == self.props.home_pos:
            self.battery_remain = self.props.battery_capacity



    def try_move_to(self, target:Node) -> bool: #ターゲットのチェックポイントに向かおうとする、行けたらTrue

        if self._distance(self.last_pos, target) + self._distance(target, self.props.home_pos) > self.battery_remain:
            return False

        self.move_to(target)
        return True


    def return_home(self) -> None:
        self.move_to(self.props.home_pos)


class PlanGenerator:

    def __init__(self, *,
        pathdata:PathData,
        drone_prop:DroneProperty,
        n_drones:int,
        safety_weight:float, # weight of uncertainly
        distance_weight:float, # weight of distance
    ):
        self.pathdata = pathdata
        self.drone_prop = drone_prop
        self.n_drones = n_drones
        self.safety_weight = safety_weight
        self.distance_weight = distance_weight


    def make(self, clusters_nodes):
        return Plan(self, clusters_nodes)


@total_ordering
class Plan:
#Plan classが作られるときに呼び出される関数、評価値を計算する処理
    def __init__(self, props:PlanGenerator, clusters_nodes:List[List[Node]]):
        self.clusters_nodes = clusters_nodes
        self.drones = [Drone(props.drone_prop) for _ in range(props.n_drones)]

        nodes_to_index = props.drone_prop.pathdata.nodes_to_index

        last_visit_time_on_nodes = {}
        i_drone = 0
        text = ''

        for pre_nodes in self.clusters_nodes:
            
            nodes = detour.detour_without_zero(pre_nodes)
            text += '['
            remain_nodes = copy.copy(nodes)

            while len(remain_nodes):
                drone = self.drones[i_drone]
                text += '|{}>'.format(i_drone)

                while len(remain_nodes):
                    node = remain_nodes[0]
                    if not drone.try_move_to(node): break
                    last_visit_time_on_nodes[node] = drone.elapsed_time
                    text += '{:>2} '.format(nodes_to_index[node])
                    remain_nodes.pop(0)
                        
                drone.return_home()
                i_drone = (i_drone + 1) % props.n_drones

            text += ']'



        self.total_distance = sum([drone.total_distance for drone in self.drones])
        self.whole_time = max([drone.elapsed_time   for drone in self.drones])
        safety_on_nodes = [math.exp(-0.001279214 * (self.whole_time - last_visit_time)) for last_visit_time in last_visit_time_on_nodes.values()]
        self.average_safety = sum(safety_on_nodes) / len(safety_on_nodes)

    #   normalized_distance = ((sum_distance - distance_range.start) / (distance_range.stop - distance_range.start))
        self.value = props.safety_weight * (1.0 - self.average_safety) + props.distance_weight * self.total_distance
        self.text = text


    def __lt__(self, plan):
        if not isinstance(plan, Plan): return NotImplemented
        return self.value < plan.value


    def __eq__(self, plan):
        if not isinstance(plan, Plan): return NotImplemented
        return self.value == plan.value




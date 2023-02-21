#!/usr/bin/python3


from CS312Graph import *
import time
from collections import defaultdict
import heapq as heap

class NetworkRoutingSolver:
    def __init__( self):
        pass

    def initializeNetwork(self, network):
        assert(type(network) == CS312Graph)
        self.network = network
        # keep track of previous nodes
        self.node_order = []

    # return the shortest path between self.source and self.dest
    def getShortestPath(self, destIndex):
        # set destination index
        self.dest = destIndex
        # keep track of nodes on path
        path_edges = []
        result = self.get_node_order(destIndex)
        total_length = 0
        pathExists = result[2]
        if pathExists:
            nodes = result[0]
            total_length = result[1]
            for i in range(len(nodes) - 1):
                path_edges.append((nodes[i].loc, nodes[i + 1].loc,
                                   '{:.0f}'.format(self.dist_between_related_nodes(nodes[i], nodes[i + 1]))))
            return {'cost':total_length, 'path':path_edges}
        else :
            return {'cost':float('inf'), 'path':path_edges}

    # find the shortest paths between source node and every other node
    # send the results to getShortestPath() ???
    def computeShortestPaths(self, srcIndex, use_heap=False):
        # set the source node index
        self.source = srcIndex
        t1 = time.time()
        self.initialize_node_order()
        self.dijkstra(srcIndex, use_heap)

        t2 = time.time()
        return (t2-t1)

    def dist_between_related_nodes(self, node, prevNode):
        nodeId = node.node_id
        for i in range(len(prevNode.neighbors)):
            if prevNode.neighbors[i].dest.node_id == nodeId:
                return prevNode.neighbors[i].length
        return 0.0

    def get_node_order(self, dest_index):
        source_index = self.source
        # track total length of path
        total_length = 0
        # initialize with destination node
        dest_node = self.network.nodes[dest_index]
        node_order = [dest_node]
        i = dest_index
        # run until index points to source index
        foundEntirePath = False
        while i != source_index:
            foundConnection = False
            # node
            node = self.network.nodes[i]
            # node's previous node
            prev = self.network.nodes[self.node_order[i]]
            # find the length between prev. node and node
            for j in range(len(prev.neighbors)):
                neighbor = prev.neighbors[j]
                # if we found the neighbor node pointing to node from prev node, add length and add node to list
                if neighbor.dest.node_id == i:
                    total_length += neighbor.length
                    node_order.append(prev)
                    i = self.node_order[i]
                    foundConnection = True
                    if i == source_index:
                        # we made it back to the source, there is a shortest path
                        foundEntirePath = True
                        break
                    else:
                        break
            if not foundConnection:
                break

        return node_order, total_length, foundEntirePath

    # initialize an array that keeps track of each node's previous node
    def initialize_node_order(self):
        for i in range(len(self.network.nodes)):
            self.node_order.append(0)

    # produces an array of distances from source to other nodes and
    # a map of previous nodes to find path
    def dijkstra(self, srcIndex, use_heap):
        priority_queue = []
        # a list to map each node to its previous node

        # determine which priority queue to use
        if use_heap:
            # run dijkstra's on heap
            print('dijk on heap')
            parent_map, _ = self.dijkstra_heap()
            self.set_node_order(parent_map)
            print()
        else:
            # run dijkstra's on array
            print('dijk on array')
            priority_queue = self.create_array_priority_queue()

        if not use_heap:
            while len(priority_queue) != 0:
                t1 = time.time()
                priority_queue = self.array_explore(priority_queue)
                t2 = time.time()
                print(t2-t1)
        return

    def set_node_order(self, parent_map):
        for node in parent_map:
            fromID = node.node_id
            toID = parent_map[node].node_id
            self.node_order[fromID] = toID

    def array_explore(self, pq):
        # grab the min in the list
        source = self.array_delete_min(pq)
        # explore the min node
        for i in range(len(source[1].neighbors)):
            # neighbor of min node
            neighbor = source[1].neighbors[i]
            # index of neighbor in the priority queue
            neighbor_index = self.get_array_pq_index(neighbor, pq)
            # -1 -> neighbor is already popped off array
            if neighbor_index == -1:
                # keep iterating through neighbors
                continue
            else:
                # neighbor node in pq
                neighbor_node = pq[neighbor_index]
                # distance from source to neighbor
                dist_source_neighbor = neighbor.length
                # distance from source
                dist_source = source[0]
                # current distance of neighbor
                dist_neighbor = neighbor_node[0]
                # check if we can improve the length (-1 represents infinity, or not visited)
                if dist_neighbor > (dist_source + dist_source_neighbor) or dist_neighbor == -1:
                    # set new length
                    neighbor_node[0] = dist_source + dist_source_neighbor
                    # set this source node to the neighbor's previous node
                    self.set_previous_node(source[1], neighbor_node[1])
        return pq

    # set the node's previous node to prev_node
    def set_previous_node(self, node, prev_node):
        # find node's index in the network
        prev_node_index = self.network.nodes.index(node)
        node_index = self.network.nodes.index(prev_node)
        self.node_order[node_index] = prev_node_index

    def get_array_pq_index(self, node, pq):
        for i in range(len(pq)):
            if pq[i][1].loc == node.dest.loc:
                return i
        # return -1 if this neighbor has already been popped off the queue
        return -1

    # creates a priority queue as an array
    def create_array_priority_queue(self):
        pq = []
        # insert nodes in array to -1 (infinity, or not visited yet)
        # using a list to keep track of prev node (distance, prevNode)
        for i in range(len(self.network.getNodes())):
            pq.append([-1, self.network.getNodes()[i]])

        # set the source node to zero
        pq[self.source][0] = 0

        return pq

    # finds, removes and returns node with min distance in priority_queue (pq)
    def array_delete_min(self, pq):
        # find min, must be greater than -1
        curr_min_index = 0
        curr_min_key = -1

        for i in range(len(pq)):
            key = pq[i][0]
            if curr_min_key == -1 and key > -1:
                curr_min_key = key
                curr_min_index = i
            elif -1 < key < curr_min_key:
                curr_min_index = i
                curr_min_key = pq[i][0]
        min_node = pq[curr_min_index]
        # remove min from list
        pq.remove(min_node)
        # return min
        return min_node

    def dijkstra_heap(self):
        visited = set()
        parentsMap = {}
        pq = []
        nodeCosts = defaultdict(lambda: float('inf'))
        nodeCosts[self.network.nodes[self.source]] = 0
        heap.heappush(pq, (0, self.network.nodes[self.source]))

        while pq:

            _, node = heap.heappop(pq)
            visited.add(node)

            for adjNode in node.neighbors:
                if adjNode.dest in visited:
                    continue

                newCost = nodeCosts[node] + adjNode.length
                if nodeCosts[adjNode.dest] > newCost:
                    parentsMap[adjNode.dest] = node
                    nodeCosts[adjNode.dest] = newCost
                    heap.heappush(pq, (newCost, adjNode.dest))

        return parentsMap, nodeCosts

    def get_weight(self, node, neighbor):
        for node_n in node.neighbors:
            if node_n.node_id == neighbor.dest.node_id:
                return neighbor.length

    # swap a node with its parent
    def swap(self, node_index, parent_index):
        temp = self.heap[node_index]
        self.heap[node_index] = self.heap[parent_index]
        self.heap[parent_index] = temp


#!/usr/bin/python3


from CS312Graph import *
import time


class NetworkRoutingSolver:
    def __init__( self):
        pass

    def initializeNetwork(self, network):
        assert(type(network) == CS312Graph)
        self.network = network

    # return the shortest path between self.source and self.dest
    def getShortestPath(self, destIndex):
        # set destination index
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL 
        #       NEED TO USE
        # keep track of nodes on path
        path_edges = []
        # keep track of total length of path
        total_length = 0
        # set the index that points to the source node
        src_node = self.network.nodes[self.source]
        # TODO: edges left should be the length of edges left, not 3
        edges_left = 3
        while edges_left > 0:
            # set the edge between current node and next node
            edge = src_node.neighbors[2]
            # append that edge to the path
            path_edges.append( (edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)) )
            # add the length of that edge to the total length
            total_length += edge.length
            # set the new current source node to the destination node (moving along the path)
            src_node = edge.dest
            # decrement number of edged in the total path
            edges_left -= 1
        return {'cost':total_length, 'path':path_edges}

    # find the shortest paths between source node and every other node
    # send the results to getShortestPath() ???
    def computeShortestPaths(self, srcIndex, use_heap=False):
        # set the source node index
        self.source = srcIndex
        t1 = time.time()
        dijkstra_result = self.dijkstra(srcIndex, use_heap)
        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)
        t2 = time.time()
        return (t2-t1)

    # produces an array of distances from source to other nodes and
    # a map of previous nodes to find path
    def dijkstra(self, srcIndex, use_heap):
        priority_queue = []
        # a list to map each node to its previous node

        # determine which priority queue to use
        if use_heap:
            # run dijkstra's on heap
            print('dik on heap')
        else:
            # run dijkstra's on array
            print('dik on array')
            priority_queue = self.array_priority_queue()

        while len(priority_queue) != 0:
            if use_heap:
                return
            else:
                priority_queue = self.array_explore(priority_queue)

        return []

    def array_explore(self, pq):
        # grab the min in the list
        source = self.array_delete_min(pq)
        # explore the min node
        for i in range(len(source[1].neighbors)):
            # todo: map priority queue to the delete_min node to decrement
            # todo: track previous node
            neighbor = source[1].neighbors[i]
            print(neighbor)
            # check if distance from source to neighbor is shorter than neighbor's current set distance
            # remember: -1 acts as infinity
        return

    # creates a priority queue as an array
    def array_priority_queue(self):
        pq = []
        # insert nodes in array to -1 (infinity, or not visited yet)
        # using a list to keep track of prev node (distance, prevNode)
        for i in range(len(self.network.getNodes())):
            pq.append([-1, self.network.getNodes()[i]])

        # set the source node to zero
        pq[0][0] = 0

        return pq

    # todo: maybe we don't need this for an array...
    def array_insert(self):
        return

    # finds, removes and returns node with min distance in priority_queue (pq)
    def array_delete_min(self, pq):
        # find min, must be greater than -1
        curr_min_index = 0
        curr_min_key = -1

        for i in range(len(pq)):
            if pq[i][0] >= 0 and pq[i][0] > curr_min_key:
                curr_min_index = i
                curr_min_key = pq[i][0]
        min_node = pq[curr_min_key]
        # remove min from list
        pq.remove(min_node)
        # return min
        return min_node

    # decrements the distance of a node in the priority queue (pq)
    def array_dec_index(self, index, pq):
        return

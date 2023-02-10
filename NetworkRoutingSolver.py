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
            edge = src_node.neighbors[2]
            path_edges.append( (edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)) )
            total_length += edge.length
            src_node = edge.dest
            edges_left -= 1
        return {'cost':total_length, 'path':path_edges}

    # find the shortest paths between every point and every other point
    # send the results to getShortestPath()
    def computeShortestPaths(self, srcIndex, use_heap=False):
        # set the source node index
        self.source = srcIndex
        t1 = time.time()
        if use_heap:
            # run dijkstra's on heap
            print('d on heap')
        else:
            # run dijkstra's on array
            print('d on array')
        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)
        t2 = time.time()
        return (t2-t1)

    def dijkstra(self, network, srcIndex):
        return

    def array_insert(self):
        return

    def array_delete_min(self):
        return

    def array_dec_index(self):
        return

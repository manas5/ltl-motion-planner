'''
.. module:: incremental_product
   :synopsis: Module implements incremental product automata maintenance and
   incremental solution checking.

   Note: based on lomap.product.ts_times_buchi by
        Alphan Ulusoy <alphan@bu.edu>

'''

import networkx as nx
import operator as op

from lomap.algorithms.dijkstra import source_to_target_dijkstra

from lomap import Model, Buchi
from scc import scc, initNodes


class IncrementalProduct(Model):
    '''Product automaton between a weighted transition system and a static Buchi
    automaton with incremental updating. It maintains both reachability of final
    states and strongly connected components for quick model checking.
    '''

    def __init__(self, ltlSpec):
        '''Constructor'''
        Model.__init__(self, directed=True, multi=False)

        self.ltlSpec = ltlSpec # global specification
        # construct Buchi automaton from global LTL specification
        self.buchi = Buchi(multi=False)
        self.buchi.from_formula(self.ltlSpec)
        self.proj_ts = {} # TS states to Buchi states map
        self.proj_b = {}
        # self.init = set()
        # self.final = set()
        for nodeB in self.buchi.g.nodes_iter(data=False):
            self.proj_b[nodeB] = []
        # initialize SCC algorithm
        self.initSCC()

    def addInitialState(self, node, symbols):
        ''' Adds an initial node to the product automaton.

        Note: based on lomap.product.ts_times_buchi by
              Alphan Ulusoy <alphan@bu.edu>
        '''
        # Iterate over the initial states of the Buchi automaton
        init_buchi_states = [s for init_buchi in self.buchi.init
                        for s in self.buchi.next_states(init_buchi, symbols)]
        init_pa_states = [(node, s) for s in init_buchi_states]
        self.g.add_nodes_from(init_pa_states) # add states to PA
        self.init.update([(p, 1) for p in init_pa_states]) # mark states as initial
        # mark final states in Buchi as final states in PA
        self.final.update([p for p in init_pa_states if p[1] in self.buchi.final])
        self.proj_ts[node] = set(init_buchi_states) # update projection of PA onto K
        for init_buchi in init_buchi_states:
            self.proj_b[init_buchi] = [node]
        # incremental update algorithm for SCC
        self.updateSCC()

    def update(self, E):
        '''Incrementally updates the global structure.
        Takes a PA edges and adds them to self.g, updates the self.proj_ts.
        '''
        if not E:
            return

        # add edges to product automaton, automatically adds missing nodes
        self.g.add_weighted_edges_from(E)

        # update projection of PA onto K
        _, states, _  = zip(*E)
        for nodeK, nodeB in states:
            self.proj_ts[nodeK] = self.proj_ts.get(nodeK, set()) | {nodeB}
            if nodeK not in self.proj_b[nodeB]:
                self.proj_b[nodeB].append(nodeK)
            # Mark as final if final in buchi
            if nodeB in self.buchi.final:
                self.final.add((nodeK, nodeB))

        # incremental update algorithm for SCC
        self.updateSCC(E)

    def check(self, ts, nodeS, nodeD, propD, weight, forward=False):
        ''' Checks if new node will correspond to a blocking state in the
        product automaton. If it is the case, the given edge is rejected.
        Otherwise, it returns the edges of the product automaton that are
        determined by the given edge.

        Note: It does not add the edges to the product automaton.
        '''
        assert nodeS in self.proj_ts # nodeS is in product automaton
        E = set()
        statesD = set()
        for nodeSBuchi in self.proj_ts.get(nodeS):
            stateS = (nodeS, nodeSBuchi)
            # get next Buchi nodes from the Buchi source node using propD
            nodesDBuchi = self.buchi.next_states(nodeSBuchi, propD)
            # the new reachable states
            newStatesD = [(nodeD, nodeDBuchi) for nodeDBuchi in nodesDBuchi]
            statesD.update(newStatesD)
            # append corresponding product automaton edges
            E.update(((stateS, stateD, weight) for stateD in newStatesD))

        if forward:
            return E

        if E:
            assert not forward
            ts.g.add_edge(nodeS, nodeD, weight=weight)
            # Add edges to product automaton, automatically adds missing nodes
            self.g.add_weighted_edges_from(E)

            stack = []
            stack.extend(statesD)

            while stack:
                current_state = stack.pop()
                ts_state, buchi_state = current_state
                for _, ts_next_state in ts.g.out_edges_iter([ts_state]):
                    ts_next_prop = ts.g.node[ts_next_state].get('prop')
                    for buchi_next_state in self.buchi.next_states(buchi_state,
                                                                  ts_next_prop):
                        next_state = (ts_next_state, buchi_next_state)
                        if next_state not in self.g:
                            # Add transition w/ weight
                            self.g.add_edge(current_state, next_state, weight=weight)
                            E.add((current_state, next_state, weight))
                            # Continue search from next state
                            stack.append(next_state)
                        elif next_state not in self.g[current_state]:
                            self.g.add_edge(current_state, next_state, weight=weight)
                            E.add((current_state, next_state, weight))
        return E

    def foundPolicy(self):
        '''Checks if there is a satisfying path in the product automaton.'''
        return any([(x in scc) for x in self.final
                                   for scc in self.scc if len(scc) > 1])

    def updateSCC(self, E=None):
        '''Incrementally updates the SCCs of PA.'''

        self.scc = list(nx.strongly_connected_components(self.g))

    def satisfy(self, state):
        condn = False
        edges = []
        for edge in self.buchi.g.out_edges_iter(data=True):
            if not edge[0]==state:
                continue
            edges.append(edge)
            if edge[1]==state and not reduce(op.and_,edge[2]['input']):
                condn = True
            elif reduce(op.and_,edge[2]['input']):
                condn = False
        return condn, edges 

    def reducePA(self, robot):
        queue = []
        discovered = {}

        for v_init in self.init:
            queue.append(v_init)
            discovered[v_init] = True

        while queue:
            v = queue.pop(0)
            condn, edges = self.satisfy(v[1])
            if condn:
                for ts_state in self.proj_b[v[1]]:
                    if self.g.has_edge(v,(ts_state,v[1])):
                        self.g.remove_edge(v,(ts_state,v[1]))
                        v_next = (ts_state,v[1])
                        if not discovered.get(v_next, False):
                            discovered[v_next] = True
                            queue.append(v_next)
                        if not self.g.out_edges_iter([v]):
                            self.g.remove_node(v)
                            proj_b[v[1]].remove(v[0])
                            proj_ts[v[0]].remove(v[1])
                            # if not proj_ts[v[0]]:
                            #     ts.g.remove_node(v[0])
                for edge in edges:
                    if reduce(op.and_,edge[2]['input']):
                        for ts_state in self.proj_b[edge[1]]:
                            v_next = (ts_state,edge[1])
                            self.g.add_edge(v,v_next,weight=robot.weight(v[0],ts_state))
                            # ts.g.add_edge(v[0],ts_state,weight=robot.weight(v[0],ts_state))
                            if not discovered.get(v_next, False):
                                discovered[v_next] = True
                                queue.append(v_next)
                            
            else:
                for e_next in self.g.out_edges_iter([v]):
                    if not discovered.get(e_next[1], False):
                        discovered[e_next[1]] = True
                        queue.append(e_next[1])
        return 

    def createPA(self, prefix, suffix, robot):
        E = []
        for i in range(len(prefix)-1):
            v = prefix[i]
            v_next = prefix[i+1]
            self.g.add_edge(v,v_next,weight=robot.weight(v[0],v_next[0]))
            E.append((v[0],v_next[0],robot.weight(v[0],v_next[0])))
            self.proj_ts[v_next[0]] = self.proj_ts.get(v_next[0], set()) | {v_next[1]}
            if v_next[0] not in self.proj_b[v_next[1]]:
                self.proj_b[v_next[1]].append(v_next[0])
            if v_next[1] in self.buchi.final:
                self.final.add((v_next[0], v_next[1]))


        for i in range(len(suffix)-1):
            v = suffix[i]
            v_next = suffix[i+1]
            self.g.add_edge(v,v_next,weight=robot.weight(v[0],v_next[0]))
            E.append((v[0],v_next[0],robot.weight(v[0],v_next[0])))
            self.proj_ts[v_next[0]] = self.proj_ts.get(v_next[0], set()) | {v_next[1]}
            if v_next[0] not in self.proj_b[v_next[1]]:
                self.proj_b[v_next[1]].append(v_next[0])
            if v_next[1] in self.buchi.final:
                self.final.add((v_next[0], v_next[1]))

        return E


    def initSCC(self):
        '''Initializes the SCC computation.'''
        self.scc = None

        # incremental computation
        self.sccG = nx.DiGraph()
        self.order = []
           

    def globalPolicy(self, ts=None):
        '''Computes the global policy.'''
        paths = nx.shortest_path(self.g)

        policy = None
        suffix_cost = None
        for init in self.init:
            if self.g.has_node(init):
                for final in self.final:
                    if self.g.has_node(final):
                        prefix = paths[init][final]
                        cost, suffix = source_to_target_dijkstra(self.g, final, final)
                        if prefix and len(suffix) > 2:
                            _, prefix = source_to_target_dijkstra(self.g, init,final) 
                            comp_policy = (list(prefix), list(suffix))
                            if not policy or cost < suffix_cost:
                                policy = comp_policy
                                suffix_cost = cost
        prefix, suffix = policy
        return prefix, suffix, suffix_cost




package org.opentripplanner.jane;

import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.opentripplanner.common.pqueue.BinHeap;
import org.opentripplanner.routing.algorithm.AStar;
import org.opentripplanner.routing.algorithm.TraverseVisitor;
import org.opentripplanner.routing.algorithm.strategies.RemainingWeightHeuristic;
import org.opentripplanner.routing.algorithm.strategies.SearchTerminationStrategy;
import org.opentripplanner.routing.algorithm.strategies.TrivialRemainingWeightHeuristic;
import org.opentripplanner.routing.core.RoutingContext;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.spt.ShortestPathTree;
import org.opentripplanner.util.DateUtils;
import org.opentripplanner.util.monitoring.MonitoringStore;
import org.opentripplanner.util.monitoring.MonitoringStoreFactory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.beust.jcommander.internal.Lists;

public class JaneAStar {
	private static final Logger LOG = LoggerFactory.getLogger(JaneAStar.class);
	// FIXME this is not really a factory, it's a way to fake a global variable.
	// This should be stored at the OTPServer level.
	private static final MonitoringStore store = MonitoringStoreFactory.getStore();
	private static final double OVERSEARCH_MULTIPLIER = 4.0;

	private boolean verbose = false;
	private Map<Integer, JaneEdge> janeEdge;
	private Map<Integer, JanePoint> janePoint;
	private int type;

	private TraverseVisitor traverseVisitor;

	enum RunStatus {
		RUNNING, STOPPED
	}

	/*
	 * TODO instead of having a separate class for search state, we should just
	 * make one GenericAStar per request.
	 */
	class RunState {

		public State u;
		public ShortestPathTree spt;
		BinHeap<State> pq;
		RemainingWeightHeuristic heuristic;
		public RoutingContext rctx;
		public int nVisited;
		public List<State> targetAcceptedStates;
		public RunStatus status;
		private RoutingRequest options;
		private SearchTerminationStrategy terminationStrategy;
		public Vertex u_vertex;
		Double foundPathWeight = null;

		public RunState(RoutingRequest options, SearchTerminationStrategy terminationStrategy) {
			this.options = options;
			this.terminationStrategy = terminationStrategy;
		}
	}

	private RunState runState;

	public JaneAStar(Map<Integer, JaneEdge> janeEdge, Map<Integer, JanePoint> janePoint, int type) {
		this.janeEdge = janeEdge;
		this.janePoint = janePoint;
		this.type = type;
	}

	/**
	 * Compute SPT using default timeout and termination strategy.
	 */
	public ShortestPathTree getShortestPathTree(RoutingRequest req) {
		return getShortestPathTree(req, -1, null); // negative timeout means no
													// timeout
	}

	/**
	 * Compute SPT using default termination strategy.
	 */
	public ShortestPathTree getShortestPathTree(RoutingRequest req, double relTimeoutSeconds) {
		return this.getShortestPathTree(req, relTimeoutSeconds, null);
	}

	/** set up a single-origin search */
	public void startSearch(RoutingRequest options, SearchTerminationStrategy terminationStrategy, long abortTime) {
		startSearch(options, terminationStrategy, abortTime, true);
	}

	/**
	 * set up the search, optionally not adding the initial state to the queue
	 * (for multi-state Dijkstra)
	 */
	private void startSearch(RoutingRequest options, SearchTerminationStrategy terminationStrategy, long abortTime,
			boolean addToQueue) {

		runState = new RunState(options, terminationStrategy);
		runState.rctx = options.getRoutingContext();
		runState.spt = options.getNewShortestPathTree();

		// We want to reuse the heuristic instance in a series of requests for
		// the same target to avoid repeated work.
		runState.heuristic = runState.rctx.remainingWeightHeuristic;

		// Since initial states can be multiple, heuristic cannot depend on the
		// initial state.
		runState.heuristic.initialize(runState.options, abortTime);
		if (abortTime < Long.MAX_VALUE && System.currentTimeMillis() > abortTime) {
			LOG.warn("Timeout during initialization of goal direction heuristic.");
			options.rctx.debugOutput.timedOut = true;
			runState = null; // Search timed out
			return;
		}

		// Priority Queue.
		// NOTE(flamholz): the queue is self-resizing, so we initialize it to
		// have
		// size = O(sqrt(|V|)) << |V|. For reference, a random, undirected
		// search
		// on a uniform 2d grid will examine roughly sqrt(|V|) vertices before
		// reaching its target.
		int initialSize = runState.rctx.graph.getVertices().size();
		initialSize = (int) Math.ceil(2 * (Math.sqrt((double) initialSize + 1)));
		runState.pq = new BinHeap<State>(initialSize);
		runState.nVisited = 0;
		runState.targetAcceptedStates = Lists.newArrayList();

		if (addToQueue) {
			State initialState = new State(options);
			initialState.places = new HashSet<JanePoint>();
			runState.spt.add(initialState);
			runState.pq.insert(initialState, 0);
		}
	}

    void iterate(){
		double lat = 0, lng = 0;
		if (runState.u.getBackState() != null) {
			Vertex backVertex = runState.u.getBackState().getVertex();
			lat = backVertex.getLat();
			lng = backVertex.getLon();
		}
        Collection<Edge> edges = runState.options.arriveBy ? runState.u_vertex.getIncoming() : runState.u_vertex.getOutgoing();
        for (Edge edge : edges) {
			Vertex outgoing = edge.getToVertex();
            // TODO Use this to board multiple trips.
            for (State v = edge.traverse(runState.u); v != null; v = v.getNextResult()) {
                if (traverseVisitor != null) {
                    traverseVisitor.visitEdge(edge, v);
                }
                if (v.getBackMode() == TraverseMode.WALK && lat == outgoing.getLat() && lng == outgoing.getLon()) {
                	// Disable back track the original location
                	continue;
                }
				v.places = (HashSet<JanePoint>) runState.u.places.clone();
				v.quality = runState.u.quality;
				JaneEdge j = janeEdge.get(edge.getId());
				if (j != null) {
					for (JanePoint point : j.points) {
						if ((point.type & type) == type && !v.places.contains(point)) {
							v.quality += point.score;
							v.places.add(point);
						}
					}
				}
                // TEST: uncomment to verify that all optimisticTraverse functions are actually
                // admissible
                // State lbs = edge.optimisticTraverse(u);
                // if ( ! (lbs.getWeight() <= v.getWeight())) {
                // System.out.printf("inadmissible lower bound %f vs %f on edge %s\n",
                // lbs.getWeightDelta(), v.getWeightDelta(), edge);
                // }

                double remaining_w = runState.heuristic.estimateRemainingWeight(v);
                if (remaining_w < 0 || Double.isInfinite(remaining_w) ) {
                    continue;
                }
                double estimate = v.getWeight() + remaining_w * runState.options.heuristicWeight;

                if (verbose) {
                    System.out.println("      edge " + edge);
                    System.out.println("      " + runState.u.getWeight() + " -> " + v.getWeight()
                            + "(w) + " + remaining_w + "(heur) = " + estimate + " vert = "
                            + v.getVertex());
                }

                // avoid enqueuing useless branches 
                if (estimate > runState.options.maxWeight) {
                    // too expensive to get here
                    if (verbose)
                        System.out.println("         too expensive to reach, not enqueued. estimated weight = " + estimate);
                    continue;
                }
                if (isWorstTimeExceeded(v, runState.options)) {
                    // too much time to get here
                    if (verbose)
                        System.out.println("         too much time to reach, not enqueued. time = " + v.getTimeSeconds());
                    continue;
                }
                
                // spt.add returns true if the state is hopeful; enqueue state if it's hopeful
                // After optimization, a trip includes transit will have a different total travel time.
                // Therefore, we will miss a potentially shorter route, when v is a target state here.
                if (runState.spt.add(v)) {
                    // report to the visitor if there is one
                    if (traverseVisitor != null) {
                        traverseVisitor.visitEnqueue(v);
                    }
                    runState.pq.insert(v, estimate);
                } 
            }
        }
    }
    
    void runSearch(long abortTime){
        /* the core of the A* algorithm */
        while (!runState.pq.empty()) { // Until the priority queue is empty:
            /*
             * Terminate based on timeout?
             */
            if (abortTime < Long.MAX_VALUE  && System.currentTimeMillis() > abortTime) {
                LOG.warn("Search timeout. origin={} target={}", runState.rctx.origin, runState.rctx.target);
                // Rather than returning null to indicate that the search was aborted/timed out,
                // we instead set a flag in the routing context and return the SPT anyway. This
                // allows returning a partial list results even when a timeout occurs.
                runState.options.rctx.aborted = true; // signal search cancellation up to higher stack frames
                runState.options.rctx.debugOutput.timedOut = true; // signal timeout in debug output object

                break;
            }
            
            // interleave some heuristic-improving work (single threaded)
            runState.heuristic.doSomeWork();

            // get the lowest-weight state in the queue
            runState.u = runState.pq.extract_min();
            
            // check that this state has not been dominated
            // and mark vertex as visited
            if (!runState.spt.visit(runState.u)) {
                // state has been dominated since it was added to the priority queue, so it is
                // not in any optimal path. drop it on the floor and try the next one.
                continue;
            }
            
            if (traverseVisitor != null) {
                traverseVisitor.visitVertex(runState.u);
            }
            
            runState.u_vertex = runState.u.getVertex();
            runState.nVisited += 1;
            
            // TODO AMB: Replace isFinal with bicycle conditions in BasicPathParser
            if (runState.u_vertex == runState.rctx.target && runState.u.isFinal() && runState.u.allPathParsersAccept()) {
                runState.foundPathWeight = runState.u.getWeight();
                runState.options.rctx.debugOutput.foundPath();
                /* Break out of the search if we've found the requested number of paths. */
                if (runState.spt.getStates(runState.rctx.target).size() >= runState.options.numItineraries) {
                    LOG.debug("total vertices visited {}", runState.nVisited);
                    break;
                }
            }
            
            iterate();

            /*
             * Should we terminate the search?
             */
            if (runState.terminationStrategy != null) {
                if (runState.terminationStrategy.shouldSearchTerminate(
                    runState.rctx.origin, runState.rctx.target, runState.u, runState.spt, runState.options)) {
                    break;
                }
            }
        }
    }

	/**
	 * @return the shortest path, or null if none is found
	 */
	public ShortestPathTree getShortestPathTree(RoutingRequest options, double relTimeoutSeconds,
			SearchTerminationStrategy terminationStrategy) {
		ShortestPathTree spt = null;
		long abortTime = DateUtils.absoluteTimeout(relTimeoutSeconds);

		startSearch(options, terminationStrategy, abortTime);

		if (runState != null) {
			runSearch(abortTime);
			spt = runState.spt;
		}

		storeMemory();
		return spt;
	}

	/** Get an SPT, starting from a collection of states */
	public ShortestPathTree getShortestPathTree(RoutingRequest options, double relTimeoutSeconds,
			SearchTerminationStrategy terminationStrategy, Collection<State> initialStates) {

		ShortestPathTree spt = null;
		long abortTime = DateUtils.absoluteTimeout(relTimeoutSeconds);

		startSearch(options, terminationStrategy, abortTime, false);

		if (runState != null) {
			for (State state : initialStates) {
				runState.spt.add(state);
				// TODO: hardwired for earliest arrival
				// TODO: weights are seconds, no?
				runState.pq.insert(state, state.getElapsedTimeSeconds());
			}

			runSearch(abortTime);
			spt = runState.spt;
		}

		return spt;
	}

	private void storeMemory() {
		if (store.isMonitoring("memoryUsed")) {
			System.gc();
			long memoryUsed = Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory();
			store.setLongMax("memoryUsed", memoryUsed);
		}
	}

	private boolean isWorstTimeExceeded(State v, RoutingRequest opt) {
		return v.getTimeSeconds() > opt.worstTime;
	}

	public void setTraverseVisitor(TraverseVisitor traverseVisitor) {
		this.traverseVisitor = traverseVisitor;
	}
}

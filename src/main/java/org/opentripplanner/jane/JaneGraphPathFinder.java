package org.opentripplanner.jane;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import org.onebusaway.gtfs.model.AgencyAndId;
import org.opentripplanner.routing.algorithm.strategies.EuclideanRemainingWeightHeuristic;
import org.opentripplanner.routing.algorithm.strategies.InterleavedBidirectionalHeuristic;
import org.opentripplanner.routing.algorithm.strategies.RemainingWeightHeuristic;
import org.opentripplanner.routing.algorithm.strategies.TrivialRemainingWeightHeuristic;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.impl.GraphPathFinder;
import org.opentripplanner.routing.impl.PathWeightComparator;
import org.opentripplanner.routing.pathparser.PathParser;
import org.opentripplanner.routing.spt.DominanceFunction;
import org.opentripplanner.routing.spt.GraphPath;
import org.opentripplanner.routing.spt.ShortestPathTree;
import org.opentripplanner.standalone.Router;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.Lists;

public class JaneGraphPathFinder extends GraphPathFinder {
	private static final Logger LOG = LoggerFactory.getLogger(JaneGraphPathFinder.class);
	private static final double DEFAULT_MAX_WALK = 2000;
	private static final double CLAMP_MAX_WALK = 15000;

	Router router;
	Map<Integer, JaneEdge> janeEdge;
	Map<Integer, JanePoint> janePoint;
	int placeType;

	public JaneGraphPathFinder(Router router, Map<Integer, JaneEdge> janeEdge, Map<Integer, JanePoint> janePoint, int placeType) {
		super(router);
		this.router = router;
		this.janeEdge = janeEdge;
		this.janePoint = janePoint;
		this.placeType = placeType;
	}

	public List<GraphPath> getPaths(RoutingRequest options) {

		if (options == null) {
			LOG.error("PathService was passed a null routing request.");
			return null;
		}

		// Reuse one instance of AStar for all N requests, which are carried out
		// sequentially
		JaneAStar aStar = new JaneAStar(janeEdge, janePoint, placeType);
		if (options.rctx == null) {
			options.setRoutingContext(router.graph);
			/*
			 * Use a pathparser that constrains the search to use
			 * SimpleTransfers.
			 */
			options.rctx.pathParsers = new PathParser[] { new Parser() };
		}
		// If this Router has a GraphVisualizer attached to it, set it as a
		// callback for the AStar search
		if (router.graphVisualizer != null) {
			aStar.setTraverseVisitor(router.graphVisualizer.traverseVisitor);
			// options.disableRemainingWeightHeuristic = true; // DEBUG
		}

		// without transit, we'd just just return multiple copies of the same
		// on-street itinerary
		/*
		 * if (!options.modes.isTransit()) { options.numItineraries = 1; }
		 */
		options.dominanceFunction = new DominanceFunction() {
			/**
			 * 
			 */
			private static final long serialVersionUID = -1079032336847799758L;

			@Override
			public boolean betterOrEqual(State a, State b) {
				return (a.getTimeSeconds() <= b.getTimeSeconds()) && (a.quality >= b.quality);
			}
		};
		LOG.debug("rreq={}", options);

		RemainingWeightHeuristic heuristic;
		if (options.disableRemainingWeightHeuristic) {
			heuristic = new TrivialRemainingWeightHeuristic();
		} else if (options.modes.isTransit()) {
			// Only use the BiDi heuristic for transit.
			heuristic = new InterleavedBidirectionalHeuristic(options.rctx.graph);
		} else {
			heuristic = new EuclideanRemainingWeightHeuristic();
		}
		// heuristic = new TrivialRemainingWeightHeuristic(); // DEBUG

		options.rctx.remainingWeightHeuristic = heuristic;
		/*
		 * In RoutingRequest, maxTransfers defaults to 2. Over long distances,
		 * we may see itineraries with far more transfers. We do not expect
		 * transfer limiting to improve search times on the
		 * LongDistancePathService, so we set it to the maximum we ever expect
		 * to see. Because people may use either the traditional path services
		 * or the LongDistancePathService, we do not change the global default
		 * but override it here.
		 */
		options.maxTransfers = 4;
		options.longDistance = true;

		/*
		 * In long distance mode, maxWalk has a different meaning. It's the
		 * radius around the origin or destination within which you can walk on
		 * the streets. If no value is provided, max walk defaults to the
		 * largest double-precision float. This would cause long distance mode
		 * to do unbounded street searches and consider the whole graph
		 * walkable.
		 */
		if (options.maxWalkDistance == Double.MAX_VALUE)
			options.maxWalkDistance = DEFAULT_MAX_WALK;
		if (options.maxWalkDistance > CLAMP_MAX_WALK)
			options.maxWalkDistance = CLAMP_MAX_WALK;
		long searchBeginTime = System.currentTimeMillis();
		LOG.debug("BEGIN SEARCH");
		List<GraphPath> paths = Lists.newArrayList();
		// Set<AgencyAndId> bannedTrips = Sets.newHashSet();
		while (true) { //while (paths.size() < options.numItineraries) 
			// TODO pull all this timeout logic into a function near
			// org.opentripplanner.util.DateUtils.absoluteTimeout()
			int timeoutIndex = paths.size();
			if (timeoutIndex >= router.timeouts.length) {
				timeoutIndex = router.timeouts.length - 1;
			}
			double timeout = searchBeginTime + (router.timeouts[timeoutIndex] * 1000);
			timeout -= System.currentTimeMillis(); // absolute to relative
			timeout /= 1000; // msec to seconds
			if (timeout <= 0) {
				// must catch this case where advancing to the next (lower)
				// timeout value means the search is timed out
				// before it even begins, because a negative relative timeout
				// will mean "no timeout" in the SPT call.
				options.rctx.aborted = true;
				break;
			}
			ShortestPathTree spt = aStar.getShortestPathTree(options, timeout);
			if (spt == null) {
				LOG.warn("SPT was null."); // unknown failure
				return null;
			}
			// if (options.rctx.aborted) break; // search timed out or was gracefully aborted for some other reason.
			// No matter search timed out or not still try to get some paths.
			List<GraphPath> newPaths = spt.getPaths();
			if (newPaths.isEmpty()) {
				break;
			}
			// Find all trips used in this path and ban them for the remaining
			// searches
			for (GraphPath path : newPaths) {
				for (State state : path.states) {
					AgencyAndId tripId = state.getTripId();
					if (tripId != null)
						options.banTrip(tripId);
				}
			}
			paths.addAll(newPaths);
			LOG.debug("we have {} paths", paths.size());
			break;
		}
		LOG.debug("END SEARCH ({} msec)", System.currentTimeMillis() - searchBeginTime);
		Collections.sort(paths, new Comparator<GraphPath>() {
			@Override
			public int compare(GraphPath o1, GraphPath o2) {
				return (int) (o2.getEndTime() - o1.getEndTime());
			}
		});
		return paths;
	}
}

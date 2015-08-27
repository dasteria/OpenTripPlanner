package org.opentripplanner.jane;

import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.onebusaway.gtfs.model.AgencyAndId;
import org.opentripplanner.api.resource.GraphPathToTripPlanConverter;
import org.opentripplanner.common.model.GenericLocation;
import org.opentripplanner.routing.algorithm.strategies.EuclideanRemainingWeightHeuristic;
import org.opentripplanner.routing.algorithm.strategies.InterleavedBidirectionalHeuristic;
import org.opentripplanner.routing.algorithm.strategies.RemainingWeightHeuristic;
import org.opentripplanner.routing.algorithm.strategies.TrivialRemainingWeightHeuristic;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.error.PathNotFoundException;
import org.opentripplanner.routing.error.VertexNotFoundException;
import org.opentripplanner.routing.impl.GraphPathFinder;
import org.opentripplanner.routing.pathparser.PathParser;
import org.opentripplanner.routing.spt.DominanceFunction;
import org.opentripplanner.routing.spt.GraphPath;
import org.opentripplanner.routing.spt.ShortestPathTree;
import org.opentripplanner.standalone.Router;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

public class AndroidGraphPathFinder {
	private static final Logger LOG = LoggerFactory.getLogger(AndroidGraphPathFinder.class);
	private static final double DEFAULT_MAX_WALK = 2000;
	private static final double CLAMP_MAX_WALK = 15000;

	Router router;
	Map<Integer, JaneEdge> janeEdge;
	Map<Integer, JanePoint> janePoint;
	int placeType;

	public AndroidGraphPathFinder(Router router, Map<Integer, JaneEdge> janeEdge, Map<Integer, JanePoint> janePoint) {
		this.router = router;
		this.janeEdge = janeEdge;
		this.janePoint = janePoint;
	}

	/* Try to find N paths through the Graph */
	public void getOneHopPath(AndroidResponse response, RoutingRequest request, int stayTime, int[] categories) {

		// We used to perform a protective clone of the RoutingRequest here.
		// There is no reason to do this if we don't modify the request.
		// Any code that changes them should be performing the copy!
		Iterator<GraphPath> gpi = null;
		long endTime = request.worstTime;
		GenericLocation to = request.to;
		List<GraphPath> paths = null;
		this.placeType = categories[0];
		request.to = request.intermediatePlaces.get(0);
		try {
			paths = getPaths(request);
			if (paths == null && request.wheelchairAccessible) {
				// There are no paths that meet the user's slope restrictions.
				// Try again without slope restrictions, and warn the user in
				// the response.
				RoutingRequest relaxedRequest = request.clone();
				relaxedRequest.maxSlope = Double.MAX_VALUE;
				request.rctx.slopeRestrictionRemoved = true;
				paths = getPaths(request);
			}
		} catch (VertexNotFoundException e) {
			LOG.info("Vertex not found: " + request.from + " : " + request.to);
			throw e;
		}

		if (paths == null || paths.size() == 0) {
			LOG.debug("Path not found: " + request.from + " : " + request.to);
			request.rctx.debugOutput.finishedRendering(); // make sure we still report full search time
			throw new PathNotFoundException();
		}

		/*
		 * Detect and report that most obnoxious of bugs: path reversal
		 * asymmetry.
		 */
		gpi = paths.iterator();
		while (gpi.hasNext()) {
			GraphPath graphPath = gpi.next();
			// TODO check, is it possible that arriveBy and time are modified in-place by the search?
			if (request.arriveBy) {
				if (graphPath.states.getLast().getTimeSeconds() > request.dateTime) {
					LOG.error("A graph path arrives after the requested time. This implies a bug.");
					gpi.remove();
				}
			} else {
				if (graphPath.states.getFirst().getTimeSeconds() < request.dateTime) {
					LOG.error("A graph path leaves before the requested time. This implies a bug.");
					gpi.remove();
				}
			}
		}
		response.setFromPlan(GraphPathToTripPlanConverter.generatePlan(paths, request));
		Map<JaneWayPoint, Long> fromWayPoints = Maps.newHashMap();
		for (GraphPath path : paths) {
			for (State state : path.states) {
				JaneWayPoint temp = new JaneWayPoint(state.getVertex().getLat(), state.getVertex().getLon(),
						state.getTimeSeconds());
				if (!fromWayPoints.containsKey(temp) || fromWayPoints.get(temp) < temp.getName()) {
					fromWayPoints.put(temp, temp.getName());
				}
			}
		}
		response.setFromWayPoints(fromWayPoints.keySet());
		// next half

		List<GraphPath> pathsTwo = null;
		this.placeType = categories[1];
		request.from = request.intermediatePlaces.get(0);
		request.to = to;
		request.worstTime = endTime;
		request.dateTime = paths.get(0).getEndTime() + stayTime;
		request.rctx = null;
		try {
			pathsTwo = getPaths(request);
			if (pathsTwo == null && request.wheelchairAccessible) {
				// There are no paths that meet the user's slope restrictions.
				// Try again without slope restrictions, and warn the user in
				// the response.
				RoutingRequest relaxedRequest = request.clone();
				relaxedRequest.maxSlope = Double.MAX_VALUE;
				request.rctx.slopeRestrictionRemoved = true;
				pathsTwo = getPaths(request);
				;
			}
			request.rctx.debugOutput.finishedCalculating();
		} catch (VertexNotFoundException e) {
			LOG.info("Vertex not found: " + request.from + " : " + request.to);
			throw e;
		}

		if (pathsTwo == null || pathsTwo.size() == 0) {
			LOG.debug("Path not found: " + request.from + " : " + request.to);
			request.rctx.debugOutput.finishedRendering(); // make sure we still
															// report full
															// search time
			throw new PathNotFoundException();
		}

		/*
		 * Detect and report that most obnoxious of bugs: path reversal
		 * asymmetry.
		 */
		gpi = pathsTwo.iterator();
		while (gpi.hasNext()) {
			GraphPath graphPath = gpi.next();
			// TODO check, is it possible that arriveBy and time are modifed
			// in-place by the search?
			if (request.arriveBy) {
				if (graphPath.states.getLast().getTimeSeconds() > request.dateTime) {
					LOG.error("A graph path arrives after the requested time. This implies a bug.");
					gpi.remove();
				}
			} else {
				if (graphPath.states.getFirst().getTimeSeconds() < request.dateTime) {
					LOG.error("A graph path leaves before the requested time. This implies a bug.");
					gpi.remove();
				}
			}
		}
		response.setToPlan(GraphPathToTripPlanConverter.generatePlan(pathsTwo, request));
		Map<JaneWayPoint, Long> toWayPoints = Maps.newHashMap();
		for (GraphPath path : pathsTwo) {
			for (State state : path.states) {
				JaneWayPoint temp = new JaneWayPoint(state.getVertex().getLat(), state.getVertex().getLon(),
						state.getTimeSeconds());
				if (!toWayPoints.containsKey(temp) || toWayPoints.get(temp) < temp.getName()) {
					toWayPoints.put(temp, temp.getName());
				}
			}
		}
		response.setToWayPoints(toWayPoints.keySet());
	}

	public List<GraphPath> getPaths(RoutingRequest options) {

		if (options == null) {
			LOG.error("PathService was passed a null routing request.");
			return null;
		}

		// Reuse one instance of AStar for all N requests, which are carried out sequentially
		JaneAStar aStar = new JaneAStar(janeEdge, janePoint, placeType);
		if (options.rctx == null) {
			options.setRoutingContext(router.graph);
			/*
			 * Use a pathparser that constrains the search to use
			 * SimpleTransfers.
			 */
			options.rctx.pathParsers = new PathParser[] { new GraphPathFinder.Parser() };
		}
		// If this Router has a GraphVisualizer attached to it, set it as a callback for the AStar search
		if (router.graphVisualizer != null) {
			aStar.setTraverseVisitor(router.graphVisualizer.traverseVisitor);
			// options.disableRemainingWeightHeuristic = true; // DEBUG
		}
		options.dominanceFunction = new DominanceFunction() {
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
			return paths;
		}
		ShortestPathTree spt = aStar.getShortestPathTree(options, timeout);
		if (spt == null) {
			LOG.warn("SPT was null."); // unknown failure
			return null;
		}
		// if (options.rctx.aborted) break; // search timed out or was
		// gracefully aborted for some other reason.
		// No matter search timed out or not still try to get some paths.
		// Turn off any route optimization optimization
		List<GraphPath> newPaths = spt.getPaths(options.getRoutingContext().target, false);
		if (newPaths.isEmpty()) {
			return paths;
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

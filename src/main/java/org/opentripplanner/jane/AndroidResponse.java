package org.opentripplanner.jane;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;

import javax.ws.rs.core.UriInfo;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import org.opentripplanner.api.model.Place;
import org.opentripplanner.api.model.TripPlan;
import org.opentripplanner.api.model.error.PlannerError;
import org.opentripplanner.api.resource.DebugOutput;

/** Represents a trip planner response, will be serialized into XML or JSON by Jersey */
@XmlRootElement
public class AndroidResponse {

    /** A dictionary of the parameters provided in the request that triggered this response. */
    @XmlElement
    public HashMap<String, String> requestParameters;
    private TripPlan from;
    private TripPlan to;
    private List<JaneWayPoint> fromWayPoints = new ArrayList<>();
    private List<JaneWayPoint> toWayPoints = new ArrayList<>();
    private PlannerError error = null;

    /** Debugging and profiling information */
    public DebugOutput debugOutput = null;

    /** This no-arg constructor exists to make JAX-RS happy. */ 
    @SuppressWarnings("unused")
    private AndroidResponse() {};

    /** Construct an new response initialized with all the incoming query parameters. */
    public AndroidResponse(UriInfo info) {
        this.requestParameters = new HashMap<String, String>();
        if (info == null) { 
            // in tests where there is no HTTP request, just leave the map empty
            return;
        }
        for (Entry<String, List<String>> e : info.getQueryParameters().entrySet()) {
            // include only the first instance of each query parameter
            requestParameters.put(e.getKey(), e.getValue().get(0));
        }
    }

    // NOTE: the order the getter methods below is semi-important, in that Jersey will use the
    // same order for the elements in the JS or XML serialized response. The traditional order
    // is request params, followed by plan, followed by errors.

    /** The actual trip plan. */
    public TripPlan getFromPlan() {
        return from;
    }
    
    public TripPlan getToPlan() {
        return to;
    }

    public List<JaneWayPoint> getFromWayPoints() {
    	return fromWayPoints;
    }

    public List<JaneWayPoint> getToWayPoints() {
    	return toWayPoints;
    }

    public void setFromPlan(TripPlan plan) {
        this.from = plan;
    }

    public void setToPlan(TripPlan plan) {
        this.to = plan;
    }
    
    public void setFromWayPoints(Set<JaneWayPoint> fromWayPoints) {
        this.fromWayPoints.addAll(fromWayPoints);
    }

    public void setToWayPoints(Set<JaneWayPoint> toWayPoints) {
        this.toWayPoints.addAll(toWayPoints);
    }    
    /** The error (if any) that this response raised. */
    @XmlElement(required=false)
    public PlannerError getError() {
        return error;
    }

    public void setError(PlannerError error) {
        this.error = error;
    }
    
}
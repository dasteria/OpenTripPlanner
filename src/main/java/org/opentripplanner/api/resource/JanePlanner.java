package org.opentripplanner.api.resource;

import static org.opentripplanner.api.resource.ServerInfo.Q;

import java.util.GregorianCalendar;
import java.util.List;
import java.util.Map;
import java.util.TimeZone;

import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.Produces;
import javax.ws.rs.QueryParam;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.UriInfo;
import javax.xml.datatype.DatatypeConfigurationException;
import javax.xml.datatype.DatatypeConstants;
import javax.xml.datatype.DatatypeFactory;
import javax.xml.datatype.XMLGregorianCalendar;

import org.opentripplanner.api.common.RoutingResource;
import org.opentripplanner.api.model.TripPlan;
import org.opentripplanner.api.model.error.PlannerError;
import org.opentripplanner.jane.JaneEdge;
import org.opentripplanner.jane.JaneGraphPathFinder;
import org.opentripplanner.jane.JanePoint;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.spt.GraphPath;
import org.opentripplanner.standalone.OTPServer;
import org.opentripplanner.standalone.Router;
import org.opentripplanner.util.DateUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Path("routers/{routerId}/jane")
public class JanePlanner extends RoutingResource {
    private static final Logger LOG = LoggerFactory.getLogger(JanePlanner.class);
    
    /** The date that the trip should arrive. */
    @QueryParam("toDate")
    protected String toDate;
    
    /** The time that the trip should arrive. */
    @QueryParam("toTime")
    protected String toTime;
    
    @QueryParam("category")
    protected int category;
    
    @QueryParam("depth")
    protected int depth;

    // We inject info about the incoming request so we can include the incoming query
    // parameters in the outgoing response. This is a TriMet requirement.
    // Jersey uses @Context to inject internal types and @InjectParam or @Resource for DI objects.
    @GET
    @Produces({ MediaType.APPLICATION_JSON, MediaType.APPLICATION_XML + Q, MediaType.TEXT_XML + Q })
    public Response plan(@Context OTPServer otpServer, @Context UriInfo uriInfo) {

        /*
         * TODO: add Lang / Locale parameter, and thus get localized content (Messages & more...)
         * TODO: from/to inputs should be converted / geocoded / etc... here, and maybe send coords 
         *       or vertex ids to planner (or error back to user)
         * TODO: org.opentripplanner.routing.module.PathServiceImpl has COOORD parsing. Abstract that
         *       out so it's used here too...
         */
        
        // Create response object, containing a copy of all request parameters. Maybe they should be in the debug section of the response.
        Response response = new Response(uriInfo);
        RoutingRequest request = null;
        try {

            /* Fill in request fields from query parameters via shared superclass method, catching any errors. */
            request = super.buildRequest();
            Router router = otpServer.getRouter(request.routerId);
            
            TimeZone tz = router.graph.getTimeZone();
            if (toDate == null && toTime != null) { // Time was provided but not date
                LOG.debug("parsing ISO datetime {}", toTime);
                try {
                    // If the time query param doesn't specify a timezone, use the graph's default. See issue #1373.
                    DatatypeFactory df = javax.xml.datatype.DatatypeFactory.newInstance();
                    XMLGregorianCalendar xmlGregCal = df.newXMLGregorianCalendar(toTime);
                    GregorianCalendar gregCal = xmlGregCal.toGregorianCalendar();
                    if (xmlGregCal.getTimezone() == DatatypeConstants.FIELD_UNDEFINED) {
                        gregCal.setTimeZone(tz);
                    }
                    request.worstTime = gregCal.getTime().getTime() / 1000;
                } catch (DatatypeConfigurationException e) {
                	request.worstTime = DateUtils.toDate(toDate, toTime, tz).getTime() / 1000;
                }
            } else {
            	request.worstTime = DateUtils.toDate(toDate, toTime, tz).getTime() / 1000;
            }
            request.numItineraries = depth>0? depth : request.numItineraries; 
            /* Find some good GraphPaths through the OTP Graph. */
            
            Map<Integer, JaneEdge> janeEdge = otpServer.getGraphService().getJaneEdge(request.routerId);
            Map<Integer, JanePoint> janePoint = otpServer.getGraphService().getJanePoint(request.routerId);
            JaneGraphPathFinder gpFinder = new JaneGraphPathFinder(router, janeEdge, janePoint, getPlaceType()); // we could also get a persistent router-scoped GraphPathFinder but there's no setup cost here
            List<GraphPath> paths = gpFinder.graphPathFinderEntryPoint(request);

            /* Convert the internal GraphPaths to a TripPlan object that is included in an OTP web service Response. */
            TripPlan plan = GraphPathToTripPlanConverter.generatePlan(paths, request);
            response.setPlan(plan);

        } catch (Exception e) {
            PlannerError error = new PlannerError(e);
            if(!PlannerError.isPlanningError(e.getClass()))
                LOG.warn("Error while planning path: ", e);
            response.setError(error);
        } finally {
            if (request != null) {
                if (request.rctx != null) {
                    response.debugOutput = request.rctx.debugOutput;
                }
                request.cleanup(); // TODO verify that this cleanup step is being done on Analyst web services
            }       
        }
        return response;
    }
    
    private int getPlaceType() {
    	return category;
    }
}

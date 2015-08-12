package org.opentripplanner.jane;

import org.opentripplanner.routing.edgetype.ElevatorAlightEdge;
import org.opentripplanner.routing.edgetype.ElevatorBoardEdge;
import org.opentripplanner.routing.edgetype.PatternHop;
import org.opentripplanner.routing.edgetype.SimpleTransfer;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.edgetype.StreetTransitLink;
import org.opentripplanner.routing.graph.Edge;

import com.vividsolutions.jts.geom.Coordinate;

public class JaneEdge {
	private String vertexName;
	private int id;
	private double[][] geometry;
	private int numOfPlaces;
	private int[] places;
	private String mode;
	
	public transient JanePoint[] points;

	public JaneEdge() {
	}

	public JaneEdge(Edge e) {
		this.vertexName = e.getFromVertex().getLabel() + ',' + e.getToVertex().getLabel();
		this.id = e.getId();
		if (e.getGeometry() == null) {
			this.geometry = null;
		} else {
			Coordinate[] coordinates = e.getGeometry().getCoordinates();
			int len = coordinates.length;
			this.geometry = new double[len][2];
			for (int i = 0; i < len; i++) {
				this.geometry[i][0] = coordinates[i].y;
				this.geometry[i][1] = coordinates[i].x;
			}
		}
		this.numOfPlaces = 0;
		this.places = null;
		if (e instanceof PatternHop) {
			this.mode = ((PatternHop) e).getMode().name();
		} else if (e instanceof StreetEdge) {
			this.mode = ((StreetEdge) e).getPermission().name();
		} else if (e instanceof SimpleTransfer) {
			this.mode = "SIMPLE_TRANSFER";
		} else if (e instanceof StreetTransitLink) {
			this.mode = ((StreetTransitLink) e).getMode().name();
		} else if (e instanceof ElevatorAlightEdge) {
			this.mode = "ELEVATOR_ALIGHT";
		} else if (e instanceof ElevatorBoardEdge) {
			this.mode = "ELEVATOR_BOARD";
		} else {
			this.mode = "";
		}
	}

	public int getId() {
		return id;
	}

	public String getName() {
		return vertexName;
	}

	public double[][] getGeometry() {
		return geometry;
	}

	public int getNumOfPlaces() {
		return numOfPlaces;
	}

	public int[] getPlaces() {
		return places;
	}
	
	public String getMode() {
		return mode;
	}

	public void setId(int id) {
		this.id = id;
	}

	public void setName(String vertexName) {
		this.vertexName = vertexName;
	}

	public void setGeometry(double[][] geometry) {
		this.geometry = geometry;
	}

	public void setNumOfPlaces(int numOfPlaces) {
		this.numOfPlaces = numOfPlaces;
	}

	public void setPlaces(int[] places) {
		this.places = places;
	}
	
	public void setMode(String mode) {
		this.mode = mode;
	}
}

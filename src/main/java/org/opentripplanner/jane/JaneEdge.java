package org.opentripplanner.jane;

import org.opentripplanner.routing.edgetype.PatternHop;
import org.opentripplanner.routing.graph.Edge;

import com.vividsolutions.jts.geom.Coordinate;

public class JaneEdge {
	private String vertexName;
	private int id;
	private double[][] geometry;
	private int[] numOfPlaces;
	private String mode;

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
		this.numOfPlaces = null;
		if (e instanceof PatternHop) {
			this.mode = ((PatternHop) e).getMode().name();
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

	public int[] getNumOfPlaces() {
		return numOfPlaces;
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

	public void setNumOfPlaces(int[] numOfPlaces) {
		this.numOfPlaces = numOfPlaces;
	}

	public void setMode(String mode) {
		this.mode = mode;
	}
}

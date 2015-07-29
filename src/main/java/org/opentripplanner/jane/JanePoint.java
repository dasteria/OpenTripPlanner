package org.opentripplanner.jane;

public class JanePoint {
	private int id;
	private double x;
	private double y;
	private float score;
	private int type;

	public JanePoint() {
	}

	public int getId() {
		return id;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}
	
	public float getScore() {
		return score;
	}

	public int getType() {
		return type;
	}

	public void setId(int id) {
		this.id = id;
	}

	public void setX(double x) {
		this.x = x;
	}

	public void setY(double y) {
		this.y = y;
	}
	
	public void setScore(float score) {
		this.score = score;
	}
	
	public void setType(int type) {
		this.type = type;
	}
}

package org.opentripplanner.jane;

public class JaneWayPoint {
    /**
     * The longitude of the place.
     */
    private final int lon;
    
    /**
     * The latitude of the place.
     */
    private final int lat;
    private final long name;
    public JaneWayPoint(double lon, double lat, long name) {
        this.lon = (int) (lon * 10000000);
        this.lat = (int) (lat * 10000000);
        this.name = name;
    }
    
    public double getLon() {
    	return (double)lon / 10000000;
    }
    
    public double getLat() {
    	return (double)lat / 10000000;
    }
    
    public long getName() {
    	return name;
    }
    
    @Override
    public int hashCode()
    {
        int hash = 17;
        hash = ((hash + lon) << 5) - (hash + lon);
        hash = ((hash + lat) << 5) - (hash + lat);
        return hash;
    }
    
    @Override
    public boolean equals(Object o) {
        if (o == this) return true;
        if (!(o instanceof JaneWayPoint)) return false;
        JaneWayPoint j = (JaneWayPoint) o;
        if (lon != j.lon || lat != j.lat) return false;
        else return true;
    }
}

package airplane.g3;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;

import org.apache.log4j.Logger;

import airplane.sim.Plane;
import airplane.sim.SimulationResult;



public class RiftPlayer extends airplane.sim.Player {

	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	private boolean all_can_fly_straight = false;
	private PriorityQueue<Plane> unfinished_departures = new PriorityQueue<Plane>();
	private Map<Plane,Integer> departures = new HashMap<Plane, Integer>();
	private ArrayList<Plane> global_planes = new ArrayList<Plane>();
	private Map<Plane,Double> omegas = new HashMap<Plane,Double>();
	private Map<Plane,Double> originals = new HashMap<Plane,Double>();
	
	private int max_delay = 10;
	
	private double max_theta = 90;
	private double increment_theta = 5;
	
	
	@Override
	public String getName() {
		return "The Rift Player";
	}
	
	/*
	 * This is called at the beginning of a new simulation. 
	 * Each Plane object includes its current location (origin), destination, and
	 * current bearing, which is -1 to indicate that it's on the ground.
	 */
	@Override
	public void startNewGame(ArrayList<Plane> planes) {
		global_planes = planes;
		all_can_fly_straight = false;
		logger.info("Starting new game!");
		for (Plane p : planes){
			originals.put(p, calculateBearing(p.getLocation(),p.getDestination()));
		}
		// At the start, first see if all the planes can make their destinations in a straight line. If so, set the boolean flag so we don't mess with them in the update method :)
		SimulationResult res = startSimulation(planes, 0);
		if (res.getReason() == 0){
			all_can_fly_straight = true;
		}
		if (!all_can_fly_straight){
			Comparator<Plane> comparator = new PlaneDepartureComparator();
			unfinished_departures = new PriorityQueue<Plane>(planes.size(), comparator);
			unfinished_departures.addAll(planes);
			while (unfinished_departures.size() > 0){
				Plane p = unfinished_departures.remove();
				int time = p.getDepartureTime();
				departures.put(p, time);
				if(!headOnCollision(planes,p)){
					res = startSimulation(planes, 0);
					while(res.getReason() != 0){
						time++;
						departures.put(p, time);
						res = startSimulation(planes, 0);
					}
				}
				//if there is going to be a head on collision, try to create a curved path.
				else{
					double bearing = calculateBearing(p.getLocation(),p.getDestination());
					double max_bearing = (bearing+max_theta)%360;
					double min_bearing = (bearing-max_theta)%360;
					//start checks
					double posbearing = bearing;
					double negbearing = bearing;
					// start positive
					boolean pos = true;
					double omega;
					do{
						if(pos){
							posbearing += increment_theta;
							posbearing = posbearing%360;
							omega = getOmega(p,posbearing);
							pos = !pos;
							originals.put(p, posbearing);
						}
						else{
							negbearing -= increment_theta;
							negbearing = negbearing%360;
							omega = getOmega(p,negbearing);
							pos = !pos;
							originals.put(p, negbearing);
						}
						omegas.put(p, omega);
						res = startSimulation(planes, 0);
					}while(res.getReason() != 0 && posbearing < max_bearing && negbearing > min_bearing);
					//if it dropped out b/c couldn't find an angle, make it delay like normal.
					if(posbearing >= max_bearing || negbearing <= min_bearing){
						originals.put(p, bearing);
						omegas.remove(p);
						res = startSimulation(planes, 0);
						while(res.getReason() != 0){
							time++;
							departures.put(p, time);
							res = startSimulation(planes, 0);
						}
					}
				}
			}
		}
	}
	
	private boolean headOnCollision(ArrayList<Plane> planes, Plane p){
		boolean collision = false;
		for (Plane test : planes){
			if (test.getDestination().equals(p.getLocation()) && test.getLocation().equals(p.getDestination()) && !omegas.containsKey(test)){
				collision = true;
				break;
			}
		}
		
		return collision;
	}
	
	private double getOmega(Plane p, double bearing){
		double radius;
		double origin = calculateBearing(p.getLocation(),p.getDestination());
		double angle;
		if(Math.abs(origin - bearing) < 180)
			angle = origin - bearing;
		else
			angle = origin - 360 + bearing;
		
		double theta = 2*angle;
		
		theta = theta*Math.PI/180;
		
		double chord = Math.sqrt(Math.pow(p.getX() - p.getDestination().x,2) + Math.pow(p.getY() - p.getDestination().y,2));
		
		radius = chord/(2*Math.sin(theta));
		
		double length = radius*theta;
		
		double time = length/p.VELOCITY;
		
		double omega = theta/time;
		
		return omega*180/Math.PI;
	}
	
	/*
	 * This is called at each step of the simulation.
	 * The List of Planes represents their current location, destination, and current
	 * bearing; the bearings array just puts these all into one spot. 
	 * This method should return an updated array of bearings.
	 */
	@Override
	public double[] updatePlanes(ArrayList<Plane> planes, int round, double[] bearings) {
		//first, if they can all fly straight at their appropriate times, do that.
		if (all_can_fly_straight){
			for (int i = 0; i < planes.size(); i++) {
				Plane p = planes.get(i);
			    if (round >= p.getDepartureTime() && p.getBearing() == -1) {
					bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
			    }
			}
			return bearings;
		}
		// if they can't all fly straight, check the priority queue and progressively delay until they can.
		for (int i = 0; i < planes.size(); i++) {
			Plane p = planes.get(i);
		    if (departures.containsKey(p) && round >= departures.get(p) && p.getBearing() == -1) {
		    	if(!omegas.containsKey(p))
		    		bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
		    	else{
		    		bearings[i] = (originals.get(p) + round*omegas.get(p))%360;
		    	}
		    }
		    else if (departures.containsKey(p) && round >= departures.get(p) && omegas.containsKey(p)){
		    	double bearing = (originals.get(p) + round*omegas.get(p))%360;
	    		bearings[i] = bearing;
		    }
		}
		
		return bearings;
	}
	
	@Override
	protected double[] simulateUpdate(ArrayList<Plane> planes, int round, double[] bearings) {
		// if no plane is in the air, find the one with the earliest 
		// departure time and move that one in the right direction
		if(departures.size() == 0){
			for (int i = 0; i < planes.size(); i++) {
				Plane p = planes.get(i);
			    if (round >= p.getDepartureTime() && p.getBearing() == -1) {
					bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
			    }
			}
		}
		else{
			for (int i = 0; i < planes.size(); i++) {
				Plane p = planes.get(i);
				Plane global = global_planes.get(i);
			    if (departures.containsKey(global) && round >= departures.get(global) && p.getBearing() == -1) {
			    	if(!omegas.containsKey(global))
			    		bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
			    	else{
			    		double bearing = (originals.get(global) + round*omegas.get(global))%360;
			    		bearings[i] = bearing;
			    	}
			    }
			    else if (departures.containsKey(global) && round >= departures.get(global) && omegas.containsKey(global)){
			    	double bearing = (originals.get(global) + round*omegas.get(global))%360;
		    		bearings[i] = bearing;
			    }
			}
			for (int i = 0; i < planes.size(); i++) {
				Plane other = planes.get(i);
				Plane global = global_planes.get(i);
				if(!departures.containsKey(global) && round >= other.getDepartureTime() && other.getBearing() == -1){
			    	boolean flying = false;
			    	for (int j = 0; j < planes.size(); j++) {
			    		if(bearings[j] >= 0){
			    			flying = true;
			    			break;
			    		}
			    	}
			    	if(!flying){
			    		bearings[i] = calculateBearing(other.getLocation(), other.getDestination());
			    	}
			    }
			}
		}
		return bearings;
	}
	
	
	
	public class PlaneDepartureComparator implements Comparator<Plane>
	{
	    @Override
	    public int compare(Plane x, Plane y)
	    {
	    	if (x.equals(null) || y.equals(null)){
	    		return 0;
	    	}
	        if (x.getDepartureTime() < y.getDepartureTime())
	        {
	            return -1;
	        }
	        if (x.getDepartureTime() > y.getDepartureTime())
	        {
	            return 1;
	        }
	        return 0;
	    }
	}

}



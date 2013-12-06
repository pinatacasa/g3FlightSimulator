package airplane.g3;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;
import java.util.PriorityQueue;

import org.apache.log4j.Logger;

import airplane.g3.RiftPlayer.PlaneDepartureComparator;
import airplane.sim.Plane;
import airplane.sim.SimulationResult;



public class ImprovedPlayer extends airplane.sim.Player {

	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	private boolean all_can_fly_straight = false;
	private PriorityQueue<Plane> unfinished_departures = new PriorityQueue<Plane>();
	private Map<Plane,Integer> departures = new HashMap<Plane, Integer>();
	private ArrayList<Plane> global_planes = new ArrayList<Plane>();
	private Map<Plane,Double> originals = new HashMap<Plane,Double>(); // initial bearing from start location.
	private Map<Plane,Plane> collisions = new HashMap<Plane,Plane>(); // literally overlapping lines
	private Hashtable<Plane, Double> omegaTable = new Hashtable<Plane, Double>();
	private Hashtable<Plane, Integer> isCurve = new Hashtable<Plane, Integer>();
	private Hashtable<Plane, Integer> statusTable = new Hashtable<Plane, Integer>();

	private double increment_theta = 5;
	private int max_delay = 10;
	private double max_theta = 30;
	
	@Override
	public String getName() {
		return "The Improved Player";
	}
	
	
	/*
	 * This is called at the beginning of a new simulation. 
	 * Each Plane object includes its current location (origin), destination, and
	 * current bearing, which is -1 to indicate that it's on the ground.
	 */
	int f = 0;
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
				int mode = 1;
				
				boolean isFirst = true;
				
				int delayTime = 0;
				int omegaTime = 0;
				
				int maxDelayTime = 5;
				int maxOmegaTime = 25;
				
				double bearing = calculateBearing(p.getLocation(),p.getDestination());

				//start checks
				double posbearing = bearing;
				double negbearing = bearing;
				// start positive
				boolean pos = true;
	
				do {
					
//					if (time >= 15) {
//						
//						mode = 1;
//						omegaTime = 0;
//						posbearing = bearing;
//						negbearing = bearing;
//						maxOmegaTime = 34;
//						departures.put(p, p.getDepartureTime());
//					}
					
					if (delayTime > maxDelayTime && mode == 0) {
						
						mode = 1;
						omegaTime = 0;
						posbearing = bearing;
						negbearing = bearing;
						
//						int newTime = time - 2;
//						departures.put(p, newTime);
					}
					
					if (omegaTime > maxOmegaTime && mode == 1) {
						
						mode = 0;
						delayTime = 0;
						if (omegaTable.containsKey(p)) {
							
							omegaTable.remove(p);
						}
						
						if (isCurve.containsKey(p)) {
							
							isCurve.remove(p);
						}
//						departures.put(p, time);
					}
					
					if (mode == 0) {
						
						delayTime ++;

						if (!isFirst) {
							
							time ++;
							departures.put(p, time);
						}
						res = startSimulation(planes, 0);
						isFirst = false;
					}
					
					if (mode == 1) {
						
						omegaTime ++;
						
//						System.err.println("1 " + omegaTime);
						if(pos){
							
							posbearing = plusDelta(posbearing, increment_theta); 
							pos = !pos;
							originals.put(p, posbearing);
							isCurve.put(p, 1);
							statusTable.put(p, 1);
						}
						else{
							
							negbearing = minusDelta(negbearing, increment_theta);
							pos = !pos;
							originals.put(p, negbearing);
							isCurve.put(p, 1);
							statusTable.put(p, -1);
						}

						res = startSimulation(planes, 0);
					}
					
					System.err.println(res.getReason() + " " + mode);
				} while(res.getReason() != 0);
				
			}
		}
	}
	
	private double formatAngle(double angle) {
		
		double result = angle;
		
		while (result < 0) {
			
			result = result + (double)360;
		}
		
		while (result >= 360) {
			
			result = result - (double)360;
		}
		
		return result;
	}
	
	private double differenceOfAngles(double angle1, double angle2) {
		
		double result;
		
		double a1 = formatAngle(angle1);
		double a2 = formatAngle(angle2);
		
		double difference = Math.abs(a1 - a2);
		if (difference <= 180) {
			
			result = difference;
		} else {
			
			result = (double)360 - difference;
		}
		
		return result;
	}
	
	private double plusDelta(double ori, double delta) {
		
		double result;
		
		result = ori + delta;
		
		result = formatAngle(result);
		
		return result;
	}
	
	private double minusDelta(double ori, double delta) {
		
		double result;
		
		result = ori - delta;
		
		result = formatAngle(result);
		
		return result;
	}

	private double getOmega(Plane p, double bearing){
		double radius;
		
		double theta = bearing;
		
		theta = theta*Math.PI/180;
		
		
		double chord = p.getLocation().distance(p.getDestination());
		
		radius = chord/(2*Math.sin(theta));
		
		double length = radius * Math.PI * bearing / 90;
		
		double time = length/p.VELOCITY;
		
		double omega = 2 * bearing/time;
		
		
		
		if (omegaTable.contains(p)) {
			
			omegaTable.remove(p);
			omegaTable.put(p, omega);
		} else {
			
			omegaTable.put(p, omega);
		}

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
			if(bearings[i] == -2)
	    		continue;
			Plane p = planes.get(i);
			
		    if (departures.containsKey(p) && round >= departures.get(p) && p.getBearing() == -1) {
		    	
		    	if(!isCurve.containsKey(p)) {
		    		
		    		bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
		    	} else {
		    		
		    		double directBearing = calculateBearing(p.getLocation(), p.getDestination());
		 		    double iniBearing = originals.get(p);
		    		double angle = differenceOfAngles(directBearing, iniBearing);
		    		logger.info("Diff: " + angle);
		    		if (angle > 90) {
			    		
			    		bearings[i] = minusDelta(iniBearing, statusTable.get(p) * 10);
			    	} else {
			    		
			    		getOmega(p, angle);
			    		logger.info("The Angle is " + angle);
			    		bearings[i] = minusDelta(iniBearing, statusTable.get(p) * omegaTable.get(p));
			    	}
		    	}
		    }
		    else if (departures.containsKey(p) && round >= departures.get(p) && isCurve.containsKey(p)){

	    		double directBearing = calculateBearing(p.getLocation(), p.getDestination());
	    		
	    		double angle = differenceOfAngles(directBearing, p.getBearing());
		    	
		    	if (angle > 90) {
		    		
		    		bearings[i] = minusDelta(p.getBearing(), statusTable.get(p) * 10);
		    	} else {
		    		
		    		if (omegaTable.containsKey(p)) {
		    			
		    			double archBearing = omegaTable.get(p);
				    	double currentBearing = bearings[i];
				    	double nextBearing = minusDelta(currentBearing, statusTable.get(p) * archBearing);
				    	
				    	double diff = differenceOfAngles(directBearing, currentBearing);
				    	
				    	if (diff <= 8.5) {
				    		
				    		bearings[i] = directBearing;
				    	} else {
				    		
				    		bearings[i] = nextBearing;
				    	}
		    		} else {
		    			
		    			getOmega(p, angle);
			    		bearings[i] = minusDelta(p.getBearing(), statusTable.get(p) * omegaTable.get(p));
		    		}
		    	}
		    }
		}
		
		return bearings;
	}
	
	@Override
	protected double[] simulateUpdate(ArrayList<Plane> planes, int round, double[] bearings) {
		// if no plane is in the air, find the one with the earliest 
		// departure time and move that one in the right direction
		if(departures.size() == 0){
//			System.err.println("x");
			for (int i = 0; i < planes.size(); i++) {
				if(bearings[i] == -2)
		    		continue;
				Plane p = planes.get(i);
			    if (round >= p.getDepartureTime() && p.getBearing() == -1) {
					bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
			    }
			}
		}
		// all the logic
		else{
//			System.err.println("rtttttttttt " + departures.size());
			for (int i = 0; i < planes.size(); i++) {
				if(bearings[i] == -2)
		    		continue;
				Plane p = planes.get(i);
				Plane global = global_planes.get(i);
			    if (departures.containsKey(global) && round >= departures.get(global) && p.getBearing() == -1) { // check to see if we can legally fly
			    	
			    	if(!isCurve.containsKey(global)) {
			    		
			    		bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
			    	} else {
			    		
			    		double directBearing = calculateBearing(p.getLocation(), p.getDestination());
			 		    double iniBearing = originals.get(global);
			    		double angle = differenceOfAngles(directBearing, iniBearing);
			    		if (angle > 90) {
				    		
				    		bearings[i] = minusDelta(iniBearing, statusTable.get(global) * 10);
				    	} else {
				    		
				    		getOmega(global, angle);
				    		bearings[i] = minusDelta(iniBearing, statusTable.get(global) * omegaTable.get(global));
				    	}
			    	}
			    }
			    else if (departures.containsKey(global) && round >= departures.get(global) && isCurve.containsKey(global)){ // when planes are in flight
			    	
			    	double directBearing = calculateBearing(p.getLocation(), p.getDestination());
			    	double angle = differenceOfAngles(directBearing, p.getBearing());
			    	
			    	if (angle > 90) {
			    		
			    		bearings[i] = minusDelta(p.getBearing(), statusTable.get(p) * 10);
			    	} else {

			    		if (omegaTable.containsKey(global)) {
			    			
			    			double archBearing = omegaTable.get(global);
					    	double currentBearing = bearings[i];
					    	double nextBearing = minusDelta(currentBearing, statusTable.get(global) * archBearing);
					    	
					    	double diff = differenceOfAngles(directBearing, currentBearing);
					    	
					    	if (diff <= 8.5) {
					    		
					    		bearings[i] = directBearing;
					    	} else {
					    		
					    		bearings[i] = nextBearing;
					    	}
			    		} else {
			    			
			    			getOmega(global, angle);
				    		bearings[i] = minusDelta(p.getBearing(), statusTable.get(global) * omegaTable.get(global));
			    		}
			    	}
			    }
			    else if (!departures.containsKey(global) && round >= p.getDepartureTime() && collisions.values().contains(global)){
			    	bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
			    }
			}
			
			for (int i = 0; i < planes.size(); i++) {
//				System.err.println("y");
				Plane other = planes.get(i);
				Plane global = global_planes.get(i);
				if(!departures.containsKey(global) && round >= other.getDepartureTime() && other.getBearing() == -1){
//					System.err.println("z " + departures.size());
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



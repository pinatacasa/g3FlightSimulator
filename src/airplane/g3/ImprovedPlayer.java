package airplane.g3;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;

import org.apache.log4j.Logger;

import airplane.sim.Plane;
import airplane.sim.SimulationResult;



public class ImprovedPlayer extends airplane.sim.Player {

	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	private boolean all_can_fly_straight = false;
	private PriorityQueue<Plane> unfinished_departures = new PriorityQueue<Plane>();
	private Map<Plane,Integer> departures = new HashMap<Plane, Integer>();
	private ArrayList<Plane> global_planes = new ArrayList<Plane>();
	private Map<Plane,Double> omegas = new HashMap<Plane,Double>(); // contains keys if contains arc
	private Map<Plane,Double> originals = new HashMap<Plane,Double>(); // initial bearing from start location.
	private Map<Plane,Plane> collisions = new HashMap<Plane,Plane>(); // literally overlapping lines
	private Map<Plane, Double> stepTable = new HashMap<Plane, Double>();
	private Map<Plane, Double> finalStepTable = new HashMap<Plane, Double>();
	
	private int max_delay = 10;
	
	private double max_theta = 30;
	private double increment_theta = 5;
	
	
	@Override
	public String getName() {
		return "The Improved Player";
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
//				System.err.println("1");
				Plane p = unfinished_departures.remove();
				int time = p.getDepartureTime();
				departures.put(p, time);
//				boolean headOnCollision = headOnCollision(planes, p);
				int mode = 1;
//				if (headOnCollision) {
//					
//					mode = 1;
//				} else  {
//					
//					mode = 0;
//				}
				
				boolean isFirst = true;
				
				int delayTime = 0;
				int omegaTime = 0;
				
				int maxDelayTime = 5;
				int maxOmegaTime = 15;
				
				double bearing = calculateBearing(p.getLocation(),p.getDestination());
				double max_bearing = (bearing+max_theta)%360;
				double min_bearing = (bearing-max_theta)%360;
				if(min_bearing < 0)
					System.err.println("min_bearing is: " + min_bearing);
				//start checks
				double posbearing = bearing;
				double negbearing = bearing;
				// start positive
				boolean pos = true;
				double omega;
				
				int imegaDelay = 0;
				int exchange = 0;
				do {
					
//					if(posbearing >= max_bearing || negbearing <= min_bearing){
////						System.err.println("Bearing is: " + bearing);
////						System.err.println("Initial bearing is: " + calculateBearing(p.getLocation(),p.getDestination()));
//						originals.put(p, bearing);
//						collisions.remove(p);
//						omegas.remove(p);
//						time = p.getDepartureTime();
//						res = startSimulation(planes, 0);
//						
//						mode = 0;
//					}
					
					if (delayTime > maxDelayTime && mode == 0) {
						
						mode = 1;
						omegaTime = 0;
						posbearing = bearing;
						negbearing = bearing;
					}
					
					if (omegaTime > maxOmegaTime && mode == 1) {
						
						mode = 0;
						delayTime = 0;
						imegaDelay = 0;
//						exchange = 0;
//						time = p.getDepartureTime();
					}
					
					if (mode == 0) {
						
						delayTime ++;
						
						System.err.println("0 " + delayTime + " " + departures.get(p));
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
							posbearing += increment_theta;
							posbearing = posbearing%360;
							omega = getOmega(p,posbearing);
							pos = !pos;
							originals.put(p, posbearing);
						}
						else{
//							System.err.println("negbearing is: " + negbearing);
							negbearing -= increment_theta;
							negbearing = (negbearing)%360;
							omega = getOmega(p,(negbearing+360)%360);
							pos = !pos;
							originals.put(p, (negbearing+360)%360);
						}
						omegas.put(p, omega);
						res = startSimulation(planes, 0);
					}
					
//					System.err.println(res.getReason());
				} while(res.getReason() != 0);
				
//				if(!headOnCollision){
//					res = startSimulation(planes, 0);
//					while(res.getReason() != 0){
////						System.err.println("2");
//						// if we time out, look for angles
//						if(time - p.getDepartureTime() >= max_delay) {
//							headOnCollision = true;
//							break;
//						}
//						time++;
//						departures.put(p, time);
//						res = startSimulation(planes, 0);
//					}
//				}
				//if there is going to be a head on collision, try to create a curved path.
//				if(headOnCollision) {
//					double bearing = calculateBearing(p.getLocation(),p.getDestination());
//					double max_bearing = (bearing+max_theta)%360;
//					double min_bearing = (bearing-max_theta)%360;
//					if(min_bearing < 0)
//						System.err.println("min_bearing is: " + min_bearing);
//					//start checks
//					double posbearing = bearing;
//					double negbearing = bearing;
//					// start positive
//					boolean pos = true;
//					double omega;
//					do{
////						System.err.println("3");
//						if(pos){
//							posbearing += increment_theta;
//							posbearing = posbearing%360;
//							omega = getOmega(p,posbearing);
//							pos = !pos;
//							originals.put(p, posbearing);
//						}
//						else{
////							System.err.println("negbearing is: " + negbearing);
//							negbearing -= increment_theta;
//							negbearing = (negbearing)%360;
//							omega = getOmega(p,(negbearing+360)%360);
//							pos = !pos;
//							originals.put(p, (negbearing+360)%360);
//						}
//						omegas.put(p, omega);
//						res = startSimulation(planes, 0);
////						System.err.println("resulted in result of: " + res.getReason());
//					}while(res.getReason() != 0 && posbearing < max_bearing && negbearing > min_bearing);
//					//if it dropped out b/c couldn't find an angle, make it delay like normal.
//					if(posbearing >= max_bearing || negbearing <= min_bearing){
////						System.err.println("Bearing is: " + bearing);
////						System.err.println("Initial bearing is: " + calculateBearing(p.getLocation(),p.getDestination()));
//						originals.put(p, bearing);
//						collisions.remove(p);
//						omegas.remove(p);
//						time = p.getDepartureTime();
//						res = startSimulation(planes, 0);
//						while(res.getReason() != 0){
//							time++;
////							System.err.println("Reason is: " + res.getReason());
//							departures.put(p, time);
//							res = startSimulation(planes, 0);
//						}
//					}
//				}
			}
		}
	}
	
	private boolean headOnCollision(ArrayList<Plane> planes, Plane p){
		boolean collision = false;
		for (Plane test : planes){
			if (test.getDestination().equals(p.getLocation()) && test.getLocation().equals(p.getDestination()) && !omegas.containsKey(test)){
				collision = true;
				collisions.put(p, test);
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
		
		double theta = angle;
		
		theta = theta*Math.PI/180;
		
		double chord = Math.sqrt(Math.pow(p.getX() - p.getDestination().x,2) + Math.pow(p.getY() - p.getDestination().y,2));
		
		radius = chord/(2*Math.sin(theta));
		
		double length = radius*theta;
		
		double time = length/p.VELOCITY;
		
		double t = 2 * time;
		stepTable.put(p, t);
		finalStepTable.put(p, t);
		
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
			if(bearings[i] == -2)
	    		continue;
			Plane p = planes.get(i);
		    if (departures.containsKey(p) && round >= departures.get(p) && p.getBearing() == -1) {
		    	if(!omegas.containsKey(p))
		    		bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
		    	else{
		    		bearings[i] = (originals.get(p)+360)%360;
		    		int t = (departures.get(p) - p.getDepartureTime());
//		    		logger.info("Omega Delay is " + new Integer(t).toString());
		    		
//		    		double left = stepTable.get(p);
//		    		left = left - (double)1;
//		    		finalStepTable.put(p, left);
		    	}
		    }
		    else if (departures.containsKey(p) && round >= departures.get(p) && omegas.containsKey(p)){
		    	
		    	double bearing = ((originals.get(p) + (round-departures.get(p))*omegas.get(p))+360)%360;
	    		bearings[i] = bearing;
		    	double left = finalStepTable.get(p);
		    	
//		    	if (left < 5) {
//		    		bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
//		    	} else {
//		    		
//		    		double bearing = ((originals.get(p) + (round-departures.get(p))*omegas.get(p))+360)%360;
//		    		bearings[i] = bearing;
//		    		left = left - (double)1;
//		    		finalStepTable.put(p, left);
//		    	}	
//		    	
//		    	double bearing = (originals.get(p) + ((round-departures.get(p))*omegas.get(p))+360)%360;
////		    	double bearing = (originals.get(p) + round*omegas.get(p))%360;
//	    		bearings[i] = bearing;
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
			for (int i = 0; i < planes.size(); i++) {
				if(bearings[i] == -2)
		    		continue;
				Plane p = planes.get(i);
				Plane global = global_planes.get(i);
			    if (departures.containsKey(global) && round >= departures.get(global) && p.getBearing() == -1) { // check to see if we can legally fly
			    	if(!omegas.containsKey(global))
			    		bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
			    	else{
			    		double bearing = originals.get(global);
			    		bearings[i] = bearing;
			    		int t = (departures.get(global) - global.getDepartureTime());
			    		
//			    		double left = stepTable.get(global);
//			    		left = left - (double)1;
//			    		stepTable.put(global, left);
			    		//System.err.println("Omega Delay is " + bearings[i]);
			    	}
			    }
			    else if (departures.containsKey(global) && round >= departures.get(global) && omegas.containsKey(global)){ // when planes are in flight
			    	
			    	double bearing = ((originals.get(global) + (round-departures.get(global))*omegas.get(global))+360)%360;
		    		bearings[i] = bearing;
			    	double left = stepTable.get(global);
			    	
//			    	if (left < 5) {
//			    		bearings[i] = calculateBearing(global.getLocation(), global.getDestination());
//			    	} else {
//			    		
//			    		double bearing = ((originals.get(global) + (round-departures.get(global))*omegas.get(global))+360)%360;
//			    		bearings[i] = bearing;
//			    		left = left - (double)1;
//			    		stepTable.put(global, left);
//			    	}	
			    }
			    else if (!departures.containsKey(global) && round >= p.getDepartureTime() && collisions.values().contains(global)){
			    	bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
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


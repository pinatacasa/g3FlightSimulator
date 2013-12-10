package airplane.g3;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Iterator;
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
	private Map<Plane,Double> originals = new HashMap<Plane,Double>(); // initial bearing from start location.
	private Hashtable<Plane, Double> omegaTable = new Hashtable<Plane, Double>();
	private Hashtable<Plane, Integer> isCurve = new Hashtable<Plane, Integer>();
	private Hashtable<Plane, Integer> statusTable = new Hashtable<Plane, Integer>();
	private Hashtable<Line2D, ArrayList<Plane>> samePath = new Hashtable<Line2D, ArrayList<Plane>>();
	private Hashtable<Plane, Integer> isReady = new Hashtable<Plane, Integer>();
//	private Hashtable<Plane, Integer> departureTime = new Hashtable<Plane, Integer>();
	
	private Map<Integer,Plane> id_lookup = new HashMap<Integer,Plane>();	
	private Map<Float,ArrayList<Plane>> arrivals_to_planes = new HashMap<Float,ArrayList<Plane>>();
	private Map<Plane,Float> planes_to_arrivals = new HashMap<Plane,Float>();	
	private Map<Plane,ArrayList<Plane>> dependencyMap = new HashMap<Plane,ArrayList<Plane>>();
	
	private double increment_theta = 5;
	private int maxDelayTime = 5;
	private int maxOmegaTime = 25;
	
	@Override
	public String getName() {
		return "The Improved Player";
	}
	
	private void setisReady(Plane p, int maxNum) {
		
		isReady.put(p, 1);
		Line2D key = new Line2D.Double();
		key = null;
		boolean flag = false;
		int count = 0;
		for (Iterator it = samePath.keySet().iterator(); it.hasNext(); ) {
			
			key = (Line2D) it.next();
			ArrayList<Plane> listP = samePath.get(key);
			
			for (Plane test : listP) {
				
				if (count > maxNum) {
					return;
				}
				
				if (flag == true) {
						
					isReady.put(test, 1);
					count ++;
				}
				if (test.id == p.id) {
					
					flag = true;
				}
			}
			
			if (flag == true) {
				
				return;
			}
		}
	}

	private void setDepartureTime(Plane p, int maxNum) {
		
		int t = departures.get(p);
		int time = t + 19;
		
		Line2D key = new Line2D.Double();
		key = null;
		boolean flag = false;
		int count = 0;
		for (Iterator it = samePath.keySet().iterator(); it.hasNext(); ) {
			
			key = (Line2D) it.next();
			ArrayList<Plane> listP = samePath.get(key);
			
			for (Plane test : listP) {
				
				if (count > maxNum) {
					return;
				}
				
				if (flag == true) {
						
					int iniTime = test.getDepartureTime();
					int dependentTime = time;
					int max = iniTime > dependentTime ? iniTime : dependentTime;
					time += 19;	
					count ++;
					departures.put(test, max);
					double oriBearing = originals.get(p);
					originals.put(test, oriBearing);
					if (isCurve.containsKey(p)) {
						
						isCurve.put(test, 1);
						int status = statusTable.get(p);
						statusTable.put(test, status);		
					} else if (isCurve.containsKey(test)){
						
						isCurve.remove(test);
					}
					
				}
				if (test.id == p.id) {
					
					flag = true;
				}
			}
			
			if (flag == true) {
				
				return;
			}
		}
	}
	/*
	 * This is called at the beginning of a new simulation. 
	 * Each Plane object includes its current location (origin), destination, and
	 * current bearing, which is -1 to indicate that it's on the ground.
	 */
	int f = 0;
	int maxN = 50;
	@Override
	public void startNewGame(ArrayList<Plane> planes) {
		
		
		global_planes = planes;
		all_can_fly_straight = false;
//		logger.info("Starting new game!");
		
		int id_new = 0;
		for (Plane p : planes){
			originals.put(p, calculateBearing(p.getLocation(),p.getDestination()));
			p.id = id_new;
			id_lookup.put(p.id, p);
			id_new++;
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
			ArrayList<Plane> dependency_unfinished_departures = new ArrayList<Plane>();
			
			for (Plane p : planes){
				ArrayList<Plane> vals = new ArrayList<Plane>();
				float arrivalTime = getArrivalTime(p);
				planes_to_arrivals.put(p, arrivalTime);
				if(arrivals_to_planes.containsKey(arrivalTime)){
					vals = arrivals_to_planes.get(arrivalTime);
				}
				vals.add(p);
				arrivals_to_planes.put(arrivalTime, vals);
			}
			//create a sorted descending order of arrival times
			Float[] arrivalTimes = new Float[arrivals_to_planes.keySet().size()];
			int i=0;
			for(float k:arrivals_to_planes.keySet()){
				arrivalTimes[i] = k;
				i++;
			}
			Arrays.sort(arrivalTimes, Collections.reverseOrder());
			//wow wasn't that stupid that we had to do that since keyset() on its own was returning null?
			
			for(double t : arrivalTimes){
				float time = (float)t;
				ArrayList<Plane> tierPlanes = arrivals_to_planes.get(time);
				//go through the planes at this arrival time (should normally be one but we do this for loop to resolve conflicts) and add the departure sequence to the unfinished_dependency_departures to loop through.
				while(tierPlanes.size()>0){
					Plane earliestPlane = null;
					double minTime = Double.MAX_VALUE;
					//in this list of planes with the same arrival time, find the one with the dependency with the earliest departure. (most of the time there is only one plane in the list anyway).
					for(Plane p: tierPlanes){
						ArrayList<Plane> allDependents = getAllChildren(p);
						dependencyMap.put(p, allDependents);
						allDependents.add(p);
						for(Plane d:allDependents){
							if(d != null){
								if(d.getDepartureTime()<minTime){
									minTime = d.getDepartureTime();
									earliestPlane = p;
								}
							}
						}
					}
					//now that we have the group of planes with the earliest departure time, add them in order of lowest departure time
					//to the final list
					ArrayList<Plane> children = dependencyMap.get(earliestPlane);
					children.add(earliestPlane);
//					System.err.println(String.format("Considering plane: %s",earliestPlane.toString()));
					while(children.size()>0){
						//find a leaf
						Plane add = null;
						for(Plane p:children){
							//leaf if no dependents
							if(p.getDependencies() == null && !dependency_unfinished_departures.contains(p)){
								add = p;
								break;
							}
							//also leaf if all dependents have been added already
							if(p.getDependencies() != null && !dependency_unfinished_departures.contains(p)){
								boolean all_dependencies_in_list = true;
								for(Plane d:getAllChildren(p)){
									//if a dependent of this plane hasn't been added, don't add it
									if(!dependency_unfinished_departures.contains(d)){
										all_dependencies_in_list = false;
										break;
									}
								}
								if(all_dependencies_in_list){
									add = p;
									break;
								}
							}
						}
						//if no leaf was found and there are still children, prune it.
						if(add ==null){
							ArrayList<Plane> removeList = new ArrayList<Plane>();
							for(Plane p:children){
								if(dependency_unfinished_departures.contains(p)){
									removeList.add(p);
								}
							}
							for(Plane p:removeList){
								children.remove(p);
							}
							continue;
						}
						//add the found leaf
						dependency_unfinished_departures.add(add);
						//remove the found leaf from the consideration set
						children.remove(add);
					}
					
					//remove the considered plane (root node) from the tierplanes to find the next one
					tierPlanes.remove(earliestPlane);
					
				}
			}
			
			
			for(Plane p : dependency_unfinished_departures) {
				samePath(p);
			}
			
			if (samePath.size() == 2) {
				
				int xx = 0;
				while (dependency_unfinished_departures.size() > 0){

					Plane p = dependency_unfinished_departures.remove(0);
					
					if (isReady.containsKey(p) && isReady.get(p) == 1) {
						
						continue;
					}
					int time = p.getDepartureTime();
					departures.put(p, time);
					setDepartureTime(p, maxN);
					
					int mode = 1;
					
					boolean isFirst = true;
					
					int delayTime = 0;
					int omegaTime = 0;
					
					double bearing = calculateBearing(p.getLocation(),p.getDestination());

					//start checks
					double posbearing = bearing;
					double negbearing = bearing;
					// start positive
					boolean pos = true;
		
					do {

						if (delayTime > maxDelayTime && mode == 0) {
							
							mode = 1;
							omegaTime = 0;
							posbearing = bearing;
							negbearing = bearing;
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
							
							departures.put(p, time);
							setDepartureTime(p, maxN);
						}
						
						if (mode == 0) {
							
							delayTime ++;

							if (!isFirst) {
								
								time ++;
								departures.put(p, time);
								setDepartureTime(p, maxN);
							}
							res = startSimulation(planes, 0);
							isFirst = false;
						}
						
						if (mode == 1) {
							
							omegaTime ++;
							
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

					} while(res.getReason() != 0 && res.getReason() != 7);
					
//					setDepartureTime(p, maxN);
					setisReady(p, maxN);
					System.err.println(xx + "Shortest feasible time is: " + arrivalTimes[0]);
					xx ++;
				}
			} else {
				int xx = 0;
				while (dependency_unfinished_departures.size() > 0){

					Plane p = dependency_unfinished_departures.remove(0);
					int time = p.getDepartureTime();
					departures.put(p, time);
					
					int mode = 1;
					
					boolean isFirst = true;
					
					int delayTime = 0;
					int omegaTime = 0;
					
					double bearing = calculateBearing(p.getLocation(),p.getDestination());

					//start checks
					double posbearing = bearing;
					double negbearing = bearing;
					// start positive
					boolean pos = true;
		
					do {

						if (delayTime > maxDelayTime && mode == 0) {
							
							mode = 1;
							omegaTime = 0;
							posbearing = bearing;
							negbearing = bearing;
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
							
							departures.put(p, time);
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

					} while(res.getReason() != 0 && res.getReason() != 7);
					
					System.err.println(xx + "Shortest feasible time is: " + arrivalTimes[0]);
					xx ++;
				}
			}
			
			
			//while we have some unfinished business to attend to...
			
		}
	}
	
	private void samePath(Plane p) {
		
		ArrayList<Plane> samePathPlanes = new ArrayList<Plane>();
		Line2D line = new Line2D.Double(p.getLocation(), p.getDestination());
		Line2D key = new Line2D.Double();
		key = null;
		
		for (Iterator it = samePath.keySet().iterator(); it.hasNext(); ) {
			
			key = (Line2D) it.next();
		    
			if (key.getX1() == line.getX1() && key.getX2() == line.getX2()
					&& key.getY1() == line.getY1()
					&& key.getY2() == line.getY2()) {
				
				samePathPlanes = samePath.get(key);
				samePath.remove(key);
				break;
			}
		}
		
		samePathPlanes.add(p);
		samePath.put(line, samePathPlanes);
	}

	private ArrayList<Plane> getAllChildren(Plane p){
		ArrayList<Plane> children = new ArrayList<Plane>();
		if(p != null){
			ArrayList<Integer> childrenIds = p.getDependencies();
			if(p.getDependencies() != null){
				for(int id:childrenIds){
					if(id != p.id){
						ArrayList<Plane> dependencies = getAllChildren(id_lookup.get(id));
						children.addAll(dependencies);
						children.add(id_lookup.get(id));
					}
				}
			}
		}
		return children;
	}
	
	private float getArrivalTime(Plane p){
		if(p != null){
			if(p.getDependencies() == null || p.getDependencies().size()==0)
				return (float)p.getDepartureTime()+(float)(Math.sqrt(Math.pow((p.getX()-p.getDestination().x),2) + Math.pow((p.getY()-p.getDestination().y),2))/p.getVelocity());
			else{
				ArrayList<Plane> children = new ArrayList<Plane>();
				if(p.getDependencies()!=null){
					for(Integer id : p.getDependencies()){
						Plane c = id_lookup.get(id);
						if(!children.contains(c) && id != p.id)
							children.add(id_lookup.get(id));
					}
				}
				ArrayList<Float> times = new ArrayList<Float>();
				for(Plane child: children){
					float arrive = getArrivalTime(child);
					times.add(arrive);
				}
				float maxChildArrive = (float)0.0;
				for(Float time : times){
					if(time > maxChildArrive){
						maxChildArrive = time;
					}
				}
				return (float)Math.max((float)p.getDepartureTime(), Math.floor(maxChildArrive)+1)+(float)(Math.sqrt(Math.pow((p.getX()-p.getDestination().x),2) + Math.pow((p.getY()-p.getDestination().y),2))/p.getVelocity());
			}
		}
		return 0;
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
			
		    if (departures.containsKey(p) && round >= departures.get(p) && p.getBearing() == -1 && p.dependenciesHaveLanded(bearings)) {
		    	
		    	if(!isCurve.containsKey(p)) {
		    		
		    		bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
		    	} else {
		    		
		    		double directBearing = calculateBearing(p.getLocation(), p.getDestination());
		 		    double iniBearing = originals.get(p);
		    		double angle = differenceOfAngles(directBearing, iniBearing);
//		    		logger.info("Diff: " + angle);
		    		if (angle > 90) {
			    		
			    		bearings[i] = minusDelta(iniBearing, statusTable.get(p) * 10);
			    	} else {
			    		
			    		getOmega(p, angle);
//			    		logger.info("The Angle is " + angle);
			    		bearings[i] = minusDelta(iniBearing, statusTable.get(p) * omegaTable.get(p));
			    	}
		    	}
		    }
		    else if (departures.containsKey(p) && round >= departures.get(p) && isCurve.containsKey(p) && p.dependenciesHaveLanded(bearings)){

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
//		depatureTime.clear();
		
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
			    if (departures.containsKey(global) && round >= departures.get(global) && p.getBearing() == -1 && global.dependenciesHaveLanded(bearings)) { // check to see if we can legally fly
			    	
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
			    else if (departures.containsKey(global) && round >= departures.get(global) && isCurve.containsKey(global) && global.dependenciesHaveLanded(bearings)){ // when planes are in flight
			    	
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
//			    else if (!departures.containsKey(global) && round >= p.getDepartureTime() && collisions.values().contains(global) && global.dependenciesHaveLanded(bearings)){
//			    	bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
//			    }
			}
			
			for (int i = 0; i < planes.size(); i++) {

				Plane other = planes.get(i);
				Plane global = global_planes.get(i);
				if(!departures.containsKey(global) && round >= other.getDepartureTime() && other.getBearing() == -1 && global.dependenciesHaveLanded(bearings)){

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



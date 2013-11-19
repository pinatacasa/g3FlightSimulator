package airplane.g3;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Hashtable;

import org.apache.log4j.Logger;

import airplane.sim.Plane;

public class SafetyPlayer  extends airplane.sim.Player {

	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	private Hashtable<Plane, Point2D> startPointMap = new Hashtable<Plane, Point2D>();
	private Hashtable<Plane, Double> startAngleMap = new Hashtable<Plane, Double>();
	private Hashtable<Plane, Integer> planeStatus = new Hashtable<Plane, Integer>(); //0: Normal, 1: Evading, 2: Regression
	
	@Override
	public String getName() {
		return "Safety Player";
	}
	
	/*
	 * This is called at the beginning of a new simulation. 
	 * Each Plane object includes its current location (origin), destination, and
	 * current bearing, which is -1 to indicate that it's on the ground.
	 */
	@Override
	public void startNewGame(ArrayList<Plane> planes) {
		logger.info("Starting new game!");
		
		for (Plane p : planes) {
			
			Double startAngle = new Double(calculateBearing(p.getLocation(), p.getDestination()));
			
			startAngleMap.put(p, startAngle);
			planeStatus.put(p, new Integer(0));
		}

	}
	
	private double addAngle(double base, double increment) {

		double result;
		
		result = Math.floor(base) + increment;

		if (result < 0) {

			result = 360 + result;
		} else if (result >= 360) {

			result = result - 360;
		}

		return result;
	}
	
	private double minusAngle(double base, double increment) {

		double result;

		result = base - increment;

		if (result < 0) {

			result = 360 + result;
		} else if (result >= 360) {

			result = result - 360;
		}

		return result;
	}
	
//	private double addAngle(double original, double base, double increment) {
//		
//		double result;
//		
//		if ((original >= 0.0 && original <= 90.0) || (original >= 270 && original <= 360.0)) {
//			
//			result = base + increment;
//
//		} else {
//			
//			result = base - increment;
//		}
//		
//		if (result < 0) {
//			
//			result = 360 + result;
//		} else if (result >= 360) {
//			
//			result = result - 360;
//		}
//		
//		return result;
//	}
	
	private Point2D predictNormal(Plane p, double bearing, int roundNum) {
		
		Point2D result = p.getLocation();
		
		for (int i = 0; i < roundNum; i ++) {

			Point2D temp = calculatePoint(result, bearing);
			result = (Point2D) temp.clone();
		}

		return result;
	}
	
	private Point2D predictRegression(Plane p, double bearing, int roundNum) {
		
		Point2D result = p.getLocation();
		double ori = bearing;
		double tempOri = ori;
		for (int i = 0; i < roundNum; i ++) {
			
			tempOri = ori;
			double nextBearing = calculateBearing(p.getLocation(), p.getDestination());
			if (Math.abs((nextBearing - tempOri)) > 10) {
				
				ori = addAngle(ori ,  10);
				
			} else {
				
				ori = nextBearing;
			}
			
			Point2D temp = calculatePoint(result, ori);
			result = (Point2D) temp.clone();
		}

		return result;
	}
	
	private Point2D calculatePoint(Plane p, double bearing, int roundNum, int status) {
		
		Point2D result = p.getLocation();
		
		if (status == 0) {
			
			result = predictNormal(p, bearing, roundNum);
		} else {
			
			result = predictRegression(p, bearing, roundNum);
		}
		
		return result;
	}
	
	private Point2D calculatePoint(Point2D p, double bearing) {
		
		if (bearing == 0) {
			
			return new Point2D.Double(p.getX(), (p.getY() - 1));
		} 
		
		if (bearing == 180) {
			
			return new Point2D.Double(p.getX(), (p.getY() + 1));
		} 
		
		if (bearing == 90) {
			
			return new Point2D.Double((p.getX() + 1), p.getY());
		} 
		
		if (bearing == 270) {
			
			return new Point2D.Double((p.getX() - 1), p.getY());
		} 
		
		double x = Math.cos(Math.PI * ((bearing - 90) / 180));
		double y = Math.sin(Math.PI * ((bearing - 90) / 180));
		
		return new Point2D.Double((p.getX() + x), (p.getY() + y));
	}
	/*
	 * This is called at each step of the simulation.
	 * The List of Planes represents their current location, destination, and current
	 * bearing; the bearings array just puts these all into one spot. 
	 * This method should return an updated array of bearings.
	 */
	private int roundNum = 4;
	private double closestDis = 7.7;
	private double degreeDelta = 9;
	@Override
	public double[] updatePlanes(ArrayList<Plane> planes, int round, double[] bearings) {
		
		// if no plane is in the air, find the one with the earliest 
		// departure time and move that one in the right direction
		
		int size = planes.size();
		int[] flag = new int[size];
		
		for (int i = 0; i < size; i++) {
			
			flag[i] = 0;
		}
		for (int i = 0; i < planes.size(); i++) {
			Plane p = planes.get(i);
			Double startAngle = startAngleMap.get(p);
		    if (p.getDepartureTime() <= round && p.getBearing() == -1) {
				
		    	
		    	
		    	Point2D currentPoint1 = p.getLocation();
		    	double bearing1 = calculateBearing(p.getLocation(), p.getDestination());
		    	Point2D nextPoint1 = calculatePoint(currentPoint1, bearing1);
		    	int f = 1;
		    	for (int j = 0; j < planes.size(); j++) {
		    		
					Plane p2 = planes.get(j);
					if (!p.equals(p2) && p2.getBearing() != -1 && p2.getBearing() != -2) {
						
						Point2D currentPoint2 = p2.getLocation();
				    	double bearing2 = bearings[j];
				    	Point2D nextPoint2 = calculatePoint(currentPoint2, bearing2);
				    	
						if (nextPoint1.distance(nextPoint2) <= 9.5 && ((bearing2 - bearing1 >= 5) || (bearing1 - bearing2 >= 5))) {
							
							f = 0;
							break;
						}
					}
		    	}
		    	
		    	if (f ==1) {
		    		
		    		bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
		    	}
//		    	double r = 0.5 / Math.sin(Math.PI * (10.0 / 180));
//		    	logger.info("Point Info 1: " + round + " "  + p.getX() + " " + bearings[i]);
		    } else if (p.getBearing() != -1 && p.getBearing() != -2) {
		    	
//		    	bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
//		    	Point2D currentPoint1 = p.getLocation();
		    	double bearing1 = bearings[i];
		    	int status1 = planeStatus.get(p); 
		    	Point2D nextPoint1 = calculatePoint(p, bearing1, roundNum, status1);
		    	
		    	for (int j = 0; j < planes.size(); j++) {
	
					Plane p2 = planes.get(j);
					if (!p.equals(p2) && p2.getBearing() != -1 && p2.getBearing() != -2) {
						
//						Point2D currentPoint2 = p2.getLocation();
				    	double bearing2 = bearings[j];
				    	int status2 = planeStatus.get(p2); 
				    	Point2D nextPoint2 = calculatePoint(p2, bearing2, roundNum, status2);
				    	
						if (nextPoint1.distance(nextPoint2) <= closestDis && ((bearing2 - bearing1 >= 5) || (bearing1 - bearing2 >= 5))) {
							
							if (flag[i] == 0) {
								double ori = bearings[i];
						    	bearings[i] = minusAngle(bearings[i] ,  10);
						    	flag[i] = 1;
						    	
						    	int tempStatus = planeStatus.get(p); 
						    	if (tempStatus != 1) {
						    		
						    		planeStatus.remove(p);
						    		planeStatus.put(p, new Integer(1));
						    	}
//								logger.info(round + ": " + i + " Crash Change: " + ori + " to " + bearings[i]  + "for " + j);
							} 
							
//							if (flag[j] == 0) {
//								double ori = bearings[j];
//						    	bearings[j] = minusAngle(bearings[j] ,  10);
//						    	flag[j] = 1;
//						    	
//						    	int tempStatus = planeStatus.get(p2); 
//						    	if (tempStatus != 1) {
//						    		
//						    		planeStatus.remove(p2);
//						    		planeStatus.put(p2, new Integer(1));
//						    	}
//								logger.info(round + ": " + j + " Crash Change: " + ori + " to " + bearings[j]  + "for " + i);
//							}
							
						}
					}
		    	}
		    	
		    	if (flag[i] == 0){
					
					double ori = bearings[i];
					double nextBearing = calculateBearing(p.getLocation(), p.getDestination());
					if (Math.abs((nextBearing - ori)) > 10) {
						
						bearings[i] = addAngle(ori ,  10);
						
//						if (nextBearing > ori)
//							bearings[i] = addAngle(ori ,  10.0);
//						else 
//							bearings[i] = minusAngle(ori ,  10.0);
						logger.info("Not enough " + ori + " to " + bearings[i] + " " + nextBearing);
					} else {
						
						bearings[i] = nextBearing;
						logger.info("Enough " + ori + " to " + bearings[i]);
					}
					flag[i] = 1;
					
					int tempStatus = planeStatus.get(p); 
			    	if (tempStatus == 1) {
			    		
			    		planeStatus.remove(p);
			    		planeStatus.put(p, new Integer(2));
			    	}
			    	
//					logger.info(round + ": " + i + " Normal Change: " + ori + " to " + bearings[i]);
				}
//		    	Double startAngle = startAngleMap.get(p);
//		    	bearings[i] = addAngle(startAngle, bearings[i] , 10.0);
//		    	double r = 0.5 / Math.sin(Math.PI * (10.0 / 180));
//		    	logger.info("Point Info 2: " + round + " "  + bearings[i] + " " + r);
		    }
		}
		
		
		return bearings;
	}
	

}

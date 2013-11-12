package airplane.g3;

import java.util.ArrayList;

import org.apache.log4j.Logger;

import airplane.sim.Plane;
import airplane.sim.SimulationResult;



public class RiftPlayer extends airplane.sim.Player {

	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	private boolean all_can_fly_straight = false;
	
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
		all_can_fly_straight = false;
		logger.info("Starting new game!");
		// At the start, first see if all the planes can make their destinations in a straight line. If so, set the boolean flag so we don't mess with them in the update method :)
		SimulationResult res = startSimulation(planes, 0);
		if (res.getReason() == 0){
			all_can_fly_straight = true;
		}
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
		// if any plane is in the air, then just keep things as-is
		for (Plane p : planes) {
		    if (p.getBearing() != -1 && p.getBearing() != -2) return bearings;
		}

		// if no plane is in the air, find the one with the earliest 
		// departure time and move that one in the right direction
		int minTime = 10000;
		int minIndex = 10000;
		for (int i = 0; i < planes.size(); i++) {
			Plane p = planes.get(i);
		    if (p.getDepartureTime() < minTime && p.getBearing() == -1) {
				minIndex = i;
				minTime = p.getDepartureTime();
		    }
		}
		
		// if it's not too early, then take off and head straight for the destination
		if (round >= minTime) {
		    Plane p = planes.get(minIndex);
		    bearings[minIndex] = calculateBearing(p.getLocation(), p.getDestination());
		}
		
		
		return bearings;
	}
	
	@Override
	protected double[] simulateUpdate(ArrayList<Plane> planes, int round, double[] bearings) {
		// if no plane is in the air, find the one with the earliest 
		// departure time and move that one in the right direction
		for (int i = 0; i < planes.size(); i++) {
			Plane p = planes.get(i);
		    if (round >= p.getDepartureTime() && p.getBearing() == -1) {
				bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
		    }
		}
		
		return bearings;
	}
	

}

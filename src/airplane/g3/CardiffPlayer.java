package airplane.g3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.concurrent.Semaphore;

import org.apache.log4j.Logger;

import airplane.sim.Plane;



public class CardiffPlayer extends airplane.sim.Player {

	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	Semaphore[][] sBoard = new Semaphore[25][25];
	HashMap<Plane, ArrayList<Semaphore>> lockMap = new HashMap<Plane, ArrayList<Semaphore>>();
	HashMap<Integer, Plane> waitMap = new HashMap<Integer, Plane>();
	
	@Override
	public String getName() {
		return "Cardiff";
	}
	
	/*
	 * This is called at the beginning of a new simulation. 
	 * Each Plane object includes its current location (origin), destination, and
	 * current bearing, which is -1 to indicate that it's on the ground.
	 */
	@Override
	public void startNewGame(ArrayList<Plane> planes) {
		logger.info("Starting new game!");
		// initialize semaphore 2D array -- represents each of the indices of the board.
		// semaphore "sections" are 4 units wide
		for (int i = 0; i < sBoard.length; i++) {
			for (int j = 0; j < sBoard[0].length; j++) {
				sBoard[i][j] = new Semaphore(1);
			}
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
		
		// all planes must release their semaphores if they have landed.
		ArrayList<Semaphore> lockSet;
		for (Plane p : planes) {
			if (p.getBearing() == -2) {
				if ((lockSet = lockMap.remove(p)) == null)
					continue;
				for (Semaphore s : lockSet)
					s.release();
			}
		}
		
		// sort planes by their time of delay
		int delay;
		for(Plane p : planes) {
			if (p.getBearing() == -2) {
				continue;
			}
			delay = round - p.getDepartureTime();
			if (delay < 0) // nothing to do here
				continue;
			waitMap.put(delay, p);
		}
		
		// do the sorting, largest delays go first...
		Integer[] allDelays = waitMap.keySet().toArray(new Integer[waitMap.keySet().size()]);
		Arrays.sort(allDelays); // sort delays from shortest to longest
		ArrayList<Plane> sortedPlanes = new ArrayList<Plane>();
		for (int i = allDelays.length-1; i >= 0; i++) {
			sortedPlanes.add(waitMap.get(allDelays[i]));
		}
		
		// now clear the waitMap for the next iteration
		waitMap.clear();
		
		// proceed to acquire semaphores
		Semaphore s;
		int sX, sY;
		ArrayList<Semaphore> locks;
		for (Plane p : planes) {
			sX = (int) Math.floor(p.getLocation().x) % 4;
			sY = (int) Math.floor(p.getLocation().y) % 4;
			locks = lockMap.get(p);
			try {
				s = sBoard[sX][sY];
				s.acquire();
				locks.add(s);
			} catch (InterruptedException e) {
				// If in here, we've tried to acquire a lock which is currently held
				// release all locks
				for (Semaphore l : locks)
					l.release();
			}
		}
		
		// after this point, each plane either has a complete set of semaphores or not.
		// those that have a full set of semaphores should move.
		
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
	

}

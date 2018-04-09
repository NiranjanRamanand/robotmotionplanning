package motionplanner;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

import motionplanner.PRM.PRMNode;


public abstract class SearchProblem extends UUSearchProblem {
	
	protected SearchNode startNode;
	int distance;
	
	protected interface SearchNode {
		public double heuristic();
		public double distance();
		public boolean goalTest();
		public ArrayList<SearchNode> getSuccessors();
		public boolean equals(Object other);
		public int hashCode();
		public void updateDistance(double newDist);
		
		
	}
	
	public abstract double distBetween(SearchNode s1, SearchNode s2);
	
	private class SearchNodeComparator implements Comparator<SearchNode>{

		@Override
		public int compare(SearchNode o1, SearchNode o2) {
			
			if(o1.heuristic() - o2.heuristic() + o1.distance() - o2.distance() > 0){
				return 1;
			} else if (o1.heuristic() - o2.heuristic() + o1.distance() - o2.distance() < 0){
				return -1;
			} else {
				return 0;
			}
		}
	}
	
	
	
	public List<SearchNode> aStarSearch(){
		PriorityQueue<SearchNode> pq = new PriorityQueue<>(20, new SearchNodeComparator());
		
		//backchain to get the path 
		HashMap<SearchNode, SearchNode> backchainMap = new HashMap<>();
		//allows shortest distances to be constantly updated
		HashMap<SearchNode, Double> distanceFromStart = new HashMap<>(); //Also functions as visited
		
		pq.add(startNode);
		distanceFromStart.put(startNode, 0.0);
		
		SearchNode goal = null;
		
		frontierLoop:
		while(!pq.isEmpty()){
			SearchNode current = pq.poll();
			//System.out.println(current);
			if(current.goalTest()){
				goal = current;
			//	System.out.println(goal.heuristic());
			//	System.out.println("INSIDE");
				break frontierLoop;
			} else {
				for(SearchNode successor : current.getSuccessors()){
					
					double newDist = distanceFromStart.get(current) + distBetween(current, successor); //successor is one away from current
					
					if(backchainMap.get(successor) == null || 
							distanceFromStart.get(successor) > newDist){
						
						distanceFromStart.put(successor, newDist);
						successor.updateDistance(newDist);
						backchainMap.put(successor, current);
						pq.add((SearchNode)successor);
						//System.out.println(successor + " " + newDist);
					}
				}
				
			}
			
		}
		//System.out.println(backchainMap.toString());
		
		return goal == null ? null : backchain(goal, backchainMap);
	}


	

	private  List<SearchNode> backchain(SearchNode goal,HashMap<SearchNode, SearchNode> backchainMap) {
		if(goal == null) return null;
		//System.out.println("exi");
		ArrayList<SearchNode> list = new ArrayList<>();
		SearchNode curr = goal;
		
		list.add(curr);

		while(!curr.equals(startNode)){
			//System.out.println(list);
			curr = backchainMap.get(curr);
			list.add(0, curr);
		}
		
		return list;
	}
		
}

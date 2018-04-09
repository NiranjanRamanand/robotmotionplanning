/*
Author:Niranjan Ramanand
Prof. Devin Balkcom gave a stub etching out class format
This is from the previous project.
*/
package motionplanner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public abstract class UUSearchProblem {

	protected int nodesExplored;
	protected int maxMemory;

	protected UUSearchNode startNode;
	
	protected interface UUSearchNode {
		public ArrayList<UUSearchNode> getSuccessors();
		public boolean goalTest();
		public int getDepth();
	}

	public List<UUSearchNode> breadthFirstSearch() {
		Queue<UUSearchNode> queue = new LinkedList<>();
		HashMap<UUSearchNode, UUSearchNode> map = new HashMap<>();
		UUSearchNode goal = null;
		
		resetStats();
		queue.add(startNode);
		map.put(startNode, startNode);//just mapping startNode to some non-null value; not especially important
		
		bfsLoop:
		while(!queue.isEmpty()){
			UUSearchNode node = queue.remove();
			updateMemory(map.size());
			incrementNodeCount();
			if(node.goalTest()){//goal found
				goal = node;
				break bfsLoop;
			} else {
				for(UUSearchNode n : node.getSuccessors()){
					if(map.get(n) == null){//not visited
						map.put(n, node);
						queue.add(n);
					}
				}
			}
		}
	
		return backchain(goal, map);
		
	}
	
	protected List<UUSearchNode> backchain(UUSearchNode node, HashMap<UUSearchNode, UUSearchNode> visited) {
		if(node == null) return null;
		
		ArrayList<UUSearchNode> list = new ArrayList<>();
		UUSearchNode curr = node;
		
		list.add(curr);

		while(!curr.equals(startNode)){
			curr = visited.get(curr);
			list.add(0,curr);
		}
		
		return list;
		
	}

	public List<UUSearchNode> depthFirstMemoizingSearch(int maxDepth) {
		HashMap<UUSearchNode, Integer> visited = new HashMap<>();
		resetStats();
		
		return dfsrm(startNode, visited, 0, maxDepth);	
	}
	
	private List<UUSearchNode> dfsrm(UUSearchNode currentNode, HashMap<UUSearchNode, Integer> visited, 
			int depth, int maxDepth) {
		incrementNodeCount();
		
		if(currentNode.goalTest()){ // base case: goal found
			List<UUSearchNode> list = new ArrayList<>();
			list.add(currentNode);
			visited.put(currentNode, depth);
			updateMemory(visited.size());
			return list;
		} else if (depth < maxDepth){ //recursive case
			visited.put(currentNode, depth);
			updateMemory(visited.size());
			
			for(UUSearchNode n : currentNode.getSuccessors()){
				if(!visited.containsKey(n)){
					List<UUSearchNode> l = dfsrm(n, visited, depth + 1, maxDepth); //recurse furthur
					if(l != null){
						l.add(currentNode);
						return l;
					}
				}
			}
		}
		return null;
	}
		
	public List<UUSearchNode> IDSearch(int maxDepth) {
		resetStats();
		
		for(int i = 0; i <= maxDepth; i++){
			List<UUSearchNode> l = depthFirstPathCheckingSearch(i);
			
			if(l != null)
				return l;
		}
		
		return null;
	}

	public List<UUSearchNode> depthFirstPathCheckingSearch(int maxDepth) {
		resetStats();
		HashSet<UUSearchNode> currentPath = new HashSet<UUSearchNode>();

		return dfsrpc(startNode, currentPath, 0, maxDepth);

	}

	private List<UUSearchNode> dfsrpc(UUSearchNode currentNode, HashSet<UUSearchNode> currentPath,
			int depth, int maxDepth) {

		incrementNodeCount();
		
		if(currentNode.goalTest()){//base case: goal found
			List<UUSearchNode> list = new ArrayList<>();
			currentPath.add(currentNode);
			updateMemory(currentPath.size()); //memory updated whenever path changes
			list.add(currentNode);
			
			
			return list;
		} else if(depth < maxDepth){//recursive case
			currentPath.add(currentNode);
			updateMemory(currentPath.size());
			
			for(UUSearchNode n : currentNode.getSuccessors()){
				if(!currentPath.contains(n)){
					List<UUSearchNode> list = dfsrpc(n, currentPath, depth + 1, maxDepth);//recurse
					
					if(list != null){
						list.add(currentNode);
						return list;
					}
				}
			}
			currentPath.remove(currentNode); //remove node when goal not found on current path
		}
		return null;
	}

	protected void resetStats() {
		nodesExplored = 0;
		maxMemory = 0;
	}
	
	protected void printStats() {
		System.out.println("Nodes explored during last search:  " + nodesExplored);
		System.out.println("Maximum memory usage during last search " + maxMemory);
	}
	
	protected void updateMemory(int currentMemory) {
		maxMemory = Math.max(currentMemory, maxMemory);
	}
	
	protected void incrementNodeCount() {
		nodesExplored++;
	}

}
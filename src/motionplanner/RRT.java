package motionplanner;

import java.awt.Rectangle;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class RRT extends SearchProblem{

	ArrayList<RRTNode> vertices = new ArrayList<>();
	HashMap<Integer, ArrayList<Integer>> edges = new HashMap<>();
	
	private double [] startCoords, goalCoords;
	private double startTheta;
	private double omega = 1, v = 200, phi = Math.PI/5;
	Obstacles obstacles;
	
	List<SearchNode> search;
	
	public RRT(double [] startCoords, double startTheta, double [] goalCoords, Obstacles obs){
		System.out.println("bounds: " + (phi*v/omega)*(phi*v/omega));
		this.startCoords = startCoords;
		this.startTheta = startTheta;
		this.goalCoords = goalCoords;
		this.obstacles = obs;
		startNode = new RRTNode(startCoords, startTheta, 0);
		vertices.add((RRTNode)startNode);
		populate(30);
		search = aStarSearch();
		System.out.println(search);
		
	}
	
	public void populate(int n){
		RRTNode node, closest, next;
		double curr = vertices.get(0).heuristic();
		
		while(vertices.size() < n){
			node = randomNode(-1);
			closest = getClosest(node);
			next = random(closest); 
			if(closest != null && next != null){
				if(isConnectable(closest.index, next, 10)){
					vertices.add(next);
		
					ArrayList<Integer> list = new ArrayList<>();
					
					list.add(closest.getIndex());
					edges.put(vertices.size() - 1, list);
					if(edges.containsKey(closest.index)){
						edges.get(closest.index).add(vertices.size() - 1);
					} else {
						ArrayList<Integer> list2 = new ArrayList<>();
						list2.add(vertices.size() - 1);
						edges.put(closest.index, list2);
					}
					
					if(next.heuristic() < curr){
						curr = next.heuristic();
						System.out.println(curr);
					}
					
					if(curr < (phi*v/omega)*phi*v/omega)
						n = 0;
				} else {
					
				}
			}
		}
		
		
		
	}

	
	
	public RRTNode getClosest(RRTNode node){
		int minIndex = -1;
		
		for(int i = 0; i < vertices.size(); i++){
			if(minIndex == -1 || distanceBetween(vertices.get(minIndex), node) > distanceBetween(vertices.get(i), node)){
				minIndex = i;
			}
		}
		
		return vertices.get(minIndex);
	}
	
	public ArrayList<RRTNode> rrtlist(RRTNode node1){
		
		double [] leftFocus = {(-v/omega)*Math.sin(node1.theta) + node1.coords[0], (v/omega)*Math.cos(node1.theta) + node1.coords[1]};
		double [] rightFocus = {(v/omega)*Math.sin(node1.theta) + node1.coords[0], (-v/omega)*Math.cos(node1.theta) + node1.coords[1]};
		
		double [] leftFocusRooted = subtractArrays(node1.coords, leftFocus);
		double [] rightFocusRooted = subtractArrays(node1.coords, rightFocus);
		
		ArrayList<RRTNode> list = new ArrayList<>();

		//Turn forward-left by phi
		list.add(new RRTNode(addArrays(rotate(phi, leftFocusRooted), node1.coords), node1.theta + phi, vertices.size() - 1));
		//Turn forward-right by phi
		list.add(new RRTNode(addArrays(rotate(-phi, rightFocusRooted), node1.coords), node1.theta - phi, vertices.size() - 1));
		//Turn backward-left by phi
		list.add(new RRTNode(addArrays(rotate(-phi, leftFocusRooted), node1.coords), node1.theta - phi, vertices.size() - 1));
		//Turn backward-right by phi
		list.add(new RRTNode(addArrays(rotate(phi, rightFocusRooted), node1.coords), node1.theta + phi, vertices.size() - 1));
		
		double dist = phi*v/omega;
		
		//move forward
		list.add(new RRTNode(addArrays(multiplyArrays(new double[] {Math.cos(node1.theta), Math.sin(node1.theta)}, dist/Math.sqrt(2)), node1.coords), node1.theta, vertices.size() - 1));
		//move backward
		list.add(new RRTNode(addArrays(multiplyArrays(new double[] {-Math.cos(node1.theta),-Math.sin(node1.theta)}, dist/Math.sqrt(2)), node1.coords), node1.theta, vertices.size() - 1));
					
		return list;
		
	}
	
	public RRTNode random(RRTNode node){
		return rrtlist(node).get((int)(Math.random()*rrtlist(node).size()));
	}
	
	public RRTNode closestOfSix(RRTNode node1, RRTNode node2){
		
		double [] leftFocus = {(-v/omega)*Math.sin(node1.theta) + node1.coords[0], (v/omega)*Math.cos(node1.theta)+ node1.coords[1]};
		double [] rightFocus = {(v/omega)*Math.sin(node1.theta) + node1.coords[0], (-v/omega)*Math.cos(node1.theta) + node1.coords[1]};
		
		double [] leftFocusRooted = subtractArrays(node1.coords, leftFocus);
		double [] rightFocusRooted = subtractArrays(node1.coords, rightFocus);
		
		ArrayList<RRTNode> list = new ArrayList<>();

		//Turn forward-left by phi
		list.add(new RRTNode(addArrays(rotate(phi, leftFocusRooted), node1.coords), node1.theta + phi, vertices.size() - 1));
		//Turn forward-right by phi
		list.add(new RRTNode(addArrays(rotate(-phi, rightFocusRooted), node1.coords), node1.theta - phi, vertices.size() - 1));
		//Turn backward-left by phi
		list.add(new RRTNode(addArrays(rotate(-phi, leftFocusRooted), node1.coords), node1.theta - phi, vertices.size() - 1));
		//Turn backward-right by phi
		list.add(new RRTNode(addArrays(rotate(phi, rightFocusRooted), node1.coords), node1.theta + phi, vertices.size() - 1));
		
		double dist = phi*v/omega;
		
		//move forward
		list.add(new RRTNode(addArrays(multiplyArrays(new double[] {Math.cos(node1.theta),Math.sin(node1.theta)}, dist/Math.sqrt(2)), node1.coords), node1.theta, vertices.size() - 1));
		//move backward
		list.add(new RRTNode(addArrays(multiplyArrays(new double[] {-Math.cos(node1.theta),-Math.sin(node1.theta)}, dist/Math.sqrt(2)), node1.coords), node1.theta, vertices.size() - 1));
							
		int minIndex = -1;
		//System.out.println("Start");
		//System.out.println(node1.coords[0] + " " + node1.coords[1]);
		for(int i = 0; i < list.size(); i++){
		//	System.out.println(list.get(i).coords[0] + " " + list.get(i).coords[1]);
			if(minIndex == -1 || distanceBetween(list.get(minIndex), node2) > distanceBetween(list.get(i), node2)){
				minIndex = i;
			}
		}
		//System.out.println("Finish");
		return list.get(minIndex);
	}
	
	public boolean isConnectable(int p1, RRTNode c, int numSegments){
		double [] currPt = vertices.get(p1).coords;
		
		double [] unitVector = new double[2];
		
		unitVector = divideArray(subtractArrays(c.coords, vertices.get(p1).coords), numSegments);
		
		currPt = addArrays(currPt, unitVector);
		//Line2D line = new Line2D.Double(vertices.get(p1).coords[0], vertices.get(p1).coords[1], c.coords[0], c.coords[1]);
		System.out.println("in");
		for(int i = 1; i < numSegments; i++){
			if(currPt != null){
				if(new MobileRobot(currPt, -1).inCollision(obstacles.obstacles)){
					System.out.println("yes");
					return false;
				}
				System.out.println(currPt[0]);
				currPt = addArrays(currPt, unitVector);
			} else {
				return false;
			}
		}System.out.println("out");
		/*
		System.out.println("line " + line.getBounds());
		for(Rectangle r : obstacles.obstacles){
			System.out.println("obs " + r.toString());
			if(r.getBounds2D().intersectsLine(line))
				return false;
			if(line.getBounds().intersects(r))
				return false;
		}*/
		return true;
	}
	
	public double [] multiplyArrays(double [] arr, double factor){
		double [] toReturn = new double[arr.length];
		
		for(int i = 0; i < arr.length; i++){
			toReturn[i] = arr[i]*factor;
		}
		
		return toReturn;
	}
	
	public double [] addArrays(double [] arr1, double [] arr2){
		double [] arr = new double[arr1.length];
		
		for(int i = 0; i < arr1.length; i++){
			arr[i] = arr1[i] + arr2[i];
		}
		
		return arr;
	}
	
	public double [] subtractArrays(double [] arr1, double [] arr2){
		double [] arr = new double[arr1.length];
		
		for(int i = 0; i < arr1.length; i++){
			arr[i] = arr1[i] - arr2[i];
		}
		
		return arr;
	}

	
	public double [] divideArray(double [] arr, int divisor){
		double [] toReturn = new double[arr.length];
		
		for(int i = 0; i < arr.length; i++){
			toReturn[i] = arr[i]/((double)divisor);
		}
		return toReturn;
	}
	
	public double distanceBetween(RRTNode n1, RRTNode n2){
		return (n1.coords[0] - n2.coords[0])*(n1.coords[0] - n2.coords[0]) + (n1.coords[1] - n2.coords[1])*(n1.coords[1] - n2.coords[1]);
	}
	
	public double distanceBetween(double [] c1, double[] c2){
		return (c1[0] - c2[0])*(c1[0] - c2[0]) + (c1[1] - c2[1])*(c1[1] - c2[1]);
	}

	public double [] rotate(double angle, double[] point){
		double [] arr = {point[0]*Math.cos(angle) - point[1]*Math.sin(angle), point[0]*Math.sin(angle) + point[1]*Math.cos(angle)};
		return arr;
	}
	
	@Override
	public double distBetween(SearchNode s1, SearchNode s2) {
		return distanceBetween(((RRTNode)s1).coords, ((RRTNode)s2).coords);
	}
	
	public RRTNode randomNode(int ind){
		double [] xy = {Math.pow(-1, (int)(2.0*Math.random())) *Math.random()*200, Math.pow(-1, (int)(2.0*Math.random())) * Math.random()*200};
		
		
		double randTheta = Math.random()*2*Math.PI;
		//System.out.println(randTheta);
		return new RRTNode(xy, randTheta, ind);
	}
	

	public class RRTNode implements SearchNode{
		double [] coords;
		double theta;
		int index;
		double distance;
		
		public RRTNode(double [] coords, double theta, int index){
			this.coords = coords;
			this.theta = theta;
			this.index = index;
		}
		
		public int getIndex(){
			return index;
		}
		@Override
		public double heuristic() {
			return (coords[0] - goalCoords[0])*(coords[0] - goalCoords[0]) + (coords[1] - goalCoords[1])*(coords[1] - goalCoords[1]);
		}

		@Override
		public double distance() {
			return distance;
		}

		@Override
		public boolean goalTest() {
			return heuristic() < (phi*v/omega)*phi*v/omega;//(phi*v/omega)*(phi*v/omega);
			
		}

		@Override
		public ArrayList<SearchNode> getSuccessors() {
			ArrayList<SearchNode> successors = new ArrayList<>();
			for(int i = 0; i < edges.get(index).size(); i++){
				successors.add(vertices.get(edges.get(index).get(i)));
			}
			
			return successors;
		}

		@Override
		public void updateDistance(double newDist) {
			distance = newDist;
			
		}
		
		@Override
		public String toString(){
			return String.valueOf(index);
		}
		
		@Override
		public boolean equals(Object other) {
			int i = ((RRTNode) other).getIndex();
			
			if(i != index)
				return false;
			
			return true;
		}
		
		@Override
		public int hashCode(){
			return index;
		}
	}
}

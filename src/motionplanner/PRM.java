package motionplanner;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;



public class PRM extends SearchProblem{
	Obstacles obstacles;
	Arm arm;
	double [] start;
	double [] finish;
	List<SearchNode> search;
	ArrayList<double[]> randomPoints = new ArrayList<>();
	ArrayList<Arm> anim;
	HashMap<Integer, ArrayList<Integer>> edges = new HashMap<>();
	
	public PRM(Obstacles obstacles, Arm arm, double [] start, double [] finish){
		this.obstacles = obstacles;
		this.arm = arm;
		this.start = start;
		this.finish = finish;
		placeRandom(10);//CHANGE 
		

		randomPoints.add(start);
		randomPoints.add(finish);
		startNode = new PRMNode(randomPoints.size() - 2);
		
		connect(12);//CHANGE
		search = aStarSearch();
		System.out.println(search);
		anim = getAnimation(15);
	}
	
	public void placeRandom(int points){
		
		int ptsPlaced = 0;
		
		while(ptsPlaced < points){
			double [] point = new double[arm.getDimension()];
			
			for(int i = 0; i < point.length; i++){
				point[i] = 2*Math.PI*Math.random();
				//System.out.println(i + " " + point[i]);
			}
			
			if(!new Arm(arm.getLinks(), point).inCollision(obstacles)){
				randomPoints.add(point);
				ptsPlaced++;
			}
		}		
	}
	
	public void connect(int k){
		for(int i = 0; i < randomPoints.size(); i++){
			edges.put(i, kNearest(i, k));
		}
	}

	public ArrayList<Integer> kNearest(int pt, int k){
		
		ArrayList<Integer> kNearest = new ArrayList<>();
		double distToJ, distToMin;
		int minIndex;
		
		for(int i = 0; i < k; i++){//ith nearest neighbor
			minIndex = -1;
			
			for(int j = 0; j < randomPoints.size(); j++){//go through all points
				//System.out.println("hiy");
				if(minIndex == -1){
					loop:
					for(int h = 0; h < randomPoints.size(); h++){
						//System.out.println(h + " " + randomPoints.size());
						if(h != pt && h != j && !kNearest.contains(h)){
							minIndex = h;
							break loop;
						}
					}
				}
					if(minIndex != -1){
					//System.out.println("hix");
					distToJ = dist(randomPoints.get(pt), randomPoints.get(j));
					distToMin = dist(randomPoints.get(pt), randomPoints.get(minIndex));
					
					if(distToJ < distToMin && !kNearest.contains(j) && j != pt && isConnectable(pt, j, 2000)){
						minIndex = j;
					}
				}
			}
			//System.out.println(pt);
			//System.out.println(pt + " " + minIndex + " " + dist(randomPoints.get(pt), randomPoints.get(minIndex)));
			if(minIndex != -1)
				kNearest.add(minIndex);
			//System.out.println(pt + " " + i + " " + minIndex);
		}
		return kNearest;
	}
	
	public boolean isConnectable(int p1, int p2, int numSegments){
		double [] currPt = randomPoints.get(p1);
		
		double [] unitVector = new double[arm.getDimension()];
		
		unitVector = divideArray(subtractArrays(randomPoints.get(p2), randomPoints.get(p1)), numSegments);
		
		currPt = addArrays(currPt, unitVector);
		
		for(int i = 1; i < numSegments; i++){
			if(new Arm(arm.getLinks(), currPt).inCollision(obstacles))
				return false;
			
			currPt = addArrays(currPt, unitVector);
		}
		
		return true;
	}
	
	public double [] divideArray(double [] arr, int divisor){
		double [] toReturn = new double[arr.length];
		
		for(int i = 0; i < arr.length; i++){
			toReturn[i] = arr[i]/divisor;
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

	public double [] multiplyArrays(double [] arr, double factor){
		double [] toReturn = new double[arr.length];
		
		for(int i = 0; i < arr.length; i++){
			toReturn[i] = arr[i]*factor;
		}
		
		return toReturn;
	}
	
	public double dist(double [] pt1, double [] pt2){
		double dist = 0;
		
		for(int i = 0; i < pt1.length; i++){
			dist += (pt1[i] - pt2[i])*(pt1[i] - pt2[i]);
		}
		//System.out.println(dist);
		return dist;
	}
	
	public ArrayList<Arm> getAnimation(int numFrames){
		ArrayList<Arm> animation = new ArrayList<>();
		
		if(search != null){
			for(int i = 0; i < search.size() - 1; i++){
				int fromNodeIndex = ((PRMNode)search.get(i)).getIndex();
				int toNodeIndex = ((PRMNode)search.get(i + 1)).getIndex();
				animation.addAll(getMovement(fromNodeIndex, toNodeIndex, 20));
			}
		}
		return animation;
	}
	
	public Arm getCurrentArm(){
		Arm currentFrame = null;
			
		if(!anim.isEmpty()){
			 currentFrame = anim.get(0);
			 if(anim.size() > 1)
				 anim.remove(0);
		}
		return currentFrame;
	}
	
	
	public ArrayList<Arm> getMovement(int start, int finish, int numFrames){
		ArrayList<Arm> movement = new ArrayList<>();
		
		double [] currPt = randomPoints.get(start);
		
		double [] unitVector = new double[arm.getDimension()];
		
		unitVector = divideArray(subtractArrays(randomPoints.get(finish), randomPoints.get(start)), numFrames);
		
		currPt = addArrays(currPt, unitVector);
		
		for(int i = 0; i < numFrames; i++){
			movement.add(new Arm(arm.getLinks(), currPt));
			currPt = addArrays(currPt, unitVector);
		}
		return movement;
		
	}
	
	
	
	protected class PRMNode implements SearchNode {
		int index;
		double distance;
		
		private PRMNode(int i){
			index = i;
		}
		
		@Override
		public double heuristic() {
			return dist(randomPoints.get(index), finish);
		}

		@Override
		public double distance() {
			return distance;
		}

		@Override
		public boolean goalTest() {
			return index == randomPoints.size() - 1;
		}

		@Override
		public ArrayList<SearchNode> getSuccessors() {
			ArrayList<SearchNode> successors = new ArrayList<>();
			for(int i = 0; i < edges.get(index).size(); i++){
				successors.add(new PRMNode(edges.get(index).get(i)));
			}
			
			return successors;
		}

		@Override
		public void updateDistance(double d) {
			distance = d;
			
		}
		
		public int getIndex(){
			return index;
		}
		@Override
		public String toString(){
			return String.valueOf(randomPoints.get(index)[0]);
		}
		
		@Override
		public boolean equals(Object other) {
			int i = ((PRMNode) other).getIndex();
			
			if(i != index)
				return false;
			
			return true;
		}
		
		@Override
		public int hashCode(){
			return index;
		}
		
	}

	
	
	@Override
	public double distBetween(SearchNode s1, SearchNode s2) {
		return dist(randomPoints.get(((PRMNode)s1).getIndex()),
					randomPoints.get(((PRMNode)s2).getIndex()));
	}
	
	
}

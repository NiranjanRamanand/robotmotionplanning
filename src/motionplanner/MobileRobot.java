package motionplanner;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Rectangle;

public class MobileRobot {
	private double [] coords;
	private double theta;
	
	public MobileRobot(double [] coords, double theta){
		this.coords = coords;
		this.theta = theta;
	}
	
	public boolean inCollision(Rectangle [] obstacles){
		
		for(Rectangle obs : obstacles){
			if(obs.contains((int)coords[0], (int)coords[1])){
				return true;
			}
		}
		return false;
	}	
	
	public boolean inCollision(Obstacles obstacles){
		return inCollision(obstacles.obstacles);
	}
	
	public void paintComponent(Graphics g) {
		
		g.fillOval((int)(425 + coords[0]), (int)(375 - coords[1]), 5, 5);
		
		
		
	
		//updateThetas();
	}
	
	public double [] getCoords(){
		return coords;
	}
	
	public double getTheta(){
		return theta;
	}
}

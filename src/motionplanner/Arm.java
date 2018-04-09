package motionplanner;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Rectangle;
import java.awt.geom.Line2D;



public class Arm {
	
	private int [] linkLengths;
	private int [][] cartesianCoords;
	private double [] angles;
	

	public Arm(int [] iLinkLengths, double [] iAngles){
		linkLengths = iLinkLengths;
		angles = iAngles;
		cartesianCoords = getArmState(angles, linkLengths);
	}
	
	private void updateThetas(){
		for(int i = 0; i < angles.length; i++){
			if(Math.random() > .97){
				if(Math.random() > .8)
					angles[i] += .01;
				else
					angles[i] -= .01;
			}
		}
		
		cartesianCoords = getArmState(angles, linkLengths);
		
	}

	private int [][] getArmState(double [] thetas, int [] linkLengths){
		int [][] armCoords = new int[thetas.length][2];
		double prevAngle = 0;
		double prevX = 0, prevY = 0;
		
		for(int i = 0; i < thetas.length; i++){
			prevAngle = (i == 0) ? 0 : thetas[i - 1];
			prevX = (i == 0) ? 0 : armCoords[i - 1][0];
			prevY = (i == 0) ? 0 : armCoords[i - 1][1];
			
			armCoords[i][0] += prevX + Math.cos(thetas[i] + prevAngle) * linkLengths[i];
			armCoords[i][1] += prevY + Math.sin(thetas[i] + prevAngle) * linkLengths[i];
		}
		return armCoords;
	}

	public boolean inCollision(Rectangle [] obstacles){
		int [] prevCoords;
		Line2D currLink;
		
		for(Rectangle obs : obstacles){
			for(int i = 0; i < cartesianCoords.length; i++){
				prevCoords = (i == 0) ? new int [] {0,0} : cartesianCoords[i-1];
				currLink = new Line2D.Double(prevCoords[0], prevCoords[1], cartesianCoords[i][0],
																		cartesianCoords[i][1]);
				
				if(obs.intersectsLine(currLink)){
					return true;
				}
			}
		}
		return false;
	}	
	
	public boolean inCollision(Obstacles obstacles){
		return inCollision(obstacles.obstacles);
	}
	
	public void paintComponent(Graphics g) {
		int [] prevCoords;
		
		g.setColor(Color.BLUE);
		g.fillOval((int)(425), (int)(375), 5, 5);
		g.drawString(Integer.toString(0), (int)(425), (int)(375));
		
		for(int i = 0; i < cartesianCoords.length; i++){
			prevCoords = (i == 0) ? new int [] {0,0} : cartesianCoords[i - 1];
			
			g.setColor(Color.RED);
			
			g.drawLine(425 + prevCoords[0], 375 -  prevCoords[1], 425 + cartesianCoords[i][0], 375 - cartesianCoords[i][1]);
			g.setColor(Color.BLUE);
			g.fillOval((int)(425 + cartesianCoords[i][0]), (int)(375 - cartesianCoords[i][1]), 5, 5);
			g.drawString(Integer.toString(i + 1), (int)(425 + cartesianCoords[i][0]), (int)(375 - cartesianCoords[i][1]));
		}
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		//updateThetas();
	}
	
	public int getDimension(){
		return angles.length;
	}

	public int [] getLinks(){
		return linkLengths;
	}
	public double[] getThetas(){
		return angles;
	}
	@Override
	public String toString(){
		String str = "";
		
		for(int i = 0; i < angles.length; i++){
			str += "  " + angles[i];
		}
		return str;
	}
}

package motionplanner;

import java.awt.Color;
import java.awt.Container;
import java.awt.Graphics;
import java.awt.Rectangle;
import java.util.ArrayList;

import javax.swing.JApplet;
import javax.swing.JPanel;

import motionplanner.RRT.RRTNode;
import motionplanner.SearchProblem.SearchNode;

public class MobileSimulator extends JApplet{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public void init() {
		Rectangle [] obstacles = new Rectangle [4];
		
		obstacles[0] = new Rectangle(10, 10, 40, 40);
		obstacles[1] = new Rectangle(-100, 40, 30, 30);
		obstacles[2] = new Rectangle(0, 20, 15, 15);
		obstacles[3] = new Rectangle(-40, 30, 15, 15);
		
		setSize(900, 700);
		Container contentPane = getContentPane();
		Obstacles obstacle = new Obstacles(obstacles);
		
		//Change disp
		Display disp = new Display(obstacle, new double []{-200.0, 20.0}, new double []{100.0, 50.0});
		contentPane.add(disp);
		//contentPane.add(disp);
		setVisible(true);
		
		//PRM prm = new PRM(obstacle, arm, anglesStart, anglesFinish);
		//RRT rrt = new RRT(new double [] {50.0, 50.0}, Math.PI, new double []{40.0, 40.0}, Math.PI, obstacle);
		while(true){
			this.repaint();
		}
		
	}
	
	public class Display extends JPanel{
		/**
		 * 
		 */
		Rectangle [] obstacles = new Rectangle [3];

		double [] start;
		double [] finish;
		Obstacles obstacle;
		RRT rrt;
		
		public Display(Obstacles obstacles, double [] start, double [] finish){
			obstacle = obstacles;
			this.start = start;
			this.finish = finish;
			rrt = new RRT(start, Math.PI, finish, obstacles);
			
			//prm = new PRM(obstacle, , anglesStart, anglesFinish);
		}
		
		private static final long serialVersionUID = 1L;
		
		public void paintComponent(Graphics g) {
			if(start != null && finish != null){
				super.paintComponent(g);
				/*Arm arm = prm.getCurrentArm();
				arm.paintComponent(g);
				if(arm.inCollision(obstacle)){
					System.out.println("COLLISION");
				}
				System.out.println(arm.toString());
				*/
				//System.out.println(1);
				
				for(RRTNode rn : rrt.vertices){
					new MobileRobot(rn.coords, rn.theta).paintComponent(g);
					
					for(RRTNode r : rrt.vertices){
						ArrayList<Integer> al = rrt.edges.get(r.getIndex());
						
						for(int i : al){
							//g.drawLine(425 + (int)r.coords[0], 375 - (int)r.coords[1], 425 + (int)rrt.vertices.get(i).coords[0], 375 - (int)rrt.vertices.get(i).coords[1]);
						}
					}
				}
				/*
				for(RRTNode r : rrt.rrtlist(rrt.randomNode(-1))){
					new MobileRobot(r.coords, r.theta).paintComponent(g);
				}
				*/
				
				if(rrt.search != null){
					g.setColor(Color.CYAN);
					double [] prevCoords = (rrt.vertices.get(0).coords);
					for(SearchNode sn: rrt.search){
						RRTNode n = ((RRTNode)sn);
						g.drawLine(425 + (int)prevCoords[0], 375 - (int)prevCoords[1], 425 + (int)n.coords[0], 375 - (int)n.coords[1]);
						new MobileRobot(n.coords, n.theta).paintComponent(g);
						prevCoords = n.coords;
						//System.out.println(n.coords[0] + " " + n.coords[1]);
					}
					g.drawLine(425 + (int)prevCoords[0], 375 - (int)prevCoords[1], 425 + (int)finish[0], 375 - (int)finish[1]);
					
				}
				g.setColor(Color.red);
				new MobileRobot(rrt.vertices.get(0).coords, rrt.vertices.get(0).theta).paintComponent(g);
				g.setColor(Color.green);
				new MobileRobot(finish, Math.PI).paintComponent(g);
				//mr.paintComponent(g);
				obstacle.paintComponent(g);
				
				repaint();
			}
		}
	}
}

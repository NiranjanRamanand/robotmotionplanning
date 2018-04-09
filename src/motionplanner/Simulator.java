package motionplanner;

import java.awt.Container;
import java.awt.Graphics;
import java.awt.Rectangle;

import javax.swing.JApplet;
import javax.swing.JPanel;


public class Simulator extends JApplet{

	private static final long serialVersionUID = 1L;

	Rectangle [] obstacles = new Rectangle [3];
	
	public void init() {
		int [] linkLengths = {100, 50, 100};
		double [] anglesStart = {Math.PI, Math.PI/2, Math.PI/2};
		double [] anglesFinish = {Math.PI, Math.PI*1.13, Math.PI};
		
		
		obstacles[0] = new Rectangle(100, 100, 10, 10);
		obstacles[1] = new Rectangle(50, 90, 10, 10);
		obstacles[2] = new Rectangle(75, 50, 5, 5);
		
		setSize(900, 700);
		Container contentPane = getContentPane();
		Obstacles obstacle = new Obstacles(obstacles);
		
		Arm arm = new Arm(linkLengths, anglesStart);
		
		Display disp = new Display(obstacle, arm, anglesStart, anglesFinish);
		contentPane.add(disp);
		setVisible(true);
		
		//PRM prm = new PRM(obstacle, arm, anglesStart, anglesFinish);
		
		while(true){
			this.repaint();
		}
		
	
		
	}
	
	public class Display extends JPanel{
		/**
		 * 
		 */
		Rectangle [] obstacles = new Rectangle [3];

		Obstacles obstacle;
		Arm arm;
		PRM prm;
		
		public Display(Obstacles obstacles, Arm arm, double [] anglesStart, double [] anglesFinish){
			obstacle = obstacles;
			this.arm = arm;
			prm = new PRM(obstacle, arm, anglesStart, anglesFinish);
		}
		
		private static final long serialVersionUID = 1L;
		
		public void paintComponent(Graphics g) {
			super.paintComponent(g);
			Arm arm = prm.getCurrentArm();
			arm.paintComponent(g);
			if(arm.inCollision(obstacle)){
				System.out.println("COLLISION");
			}
			System.out.println(arm.toString());
			
			obstacle.paintComponent(g);
			repaint();
		}
	}
	

	
}

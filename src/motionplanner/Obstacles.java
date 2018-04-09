package motionplanner;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;


public class Obstacles {
	
	@SuppressWarnings("unused")
	private static final long serialVersionUID = 1L;
	public Rectangle [] obstacles;
	
	public Obstacles(Rectangle [] obstacles){
		this.obstacles = obstacles;
	}
	
	
	public void paintComponent(Graphics g) {
		g.setColor(Color.ORANGE);
		
		Graphics2D g1 = (Graphics2D)g;
		
		for(Rectangle obstacle : obstacles){
			g1.fillRect(425 + obstacle.x, 375 - obstacle.y, obstacle.width, obstacle.height);
		}
		
		
	}
}
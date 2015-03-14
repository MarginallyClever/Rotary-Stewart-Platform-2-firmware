package RotaryStewartPlatform2;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;

import RotaryStewartPlatform2.RotaryStewartPlatform;
import RotaryStewartPlatform2.BoundingVolume;
import RotaryStewartPlatform2.Camera;
import RotaryStewartPlatform2.Cylinder;
import RotaryStewartPlatform2.IntersectionTester;
import RotaryStewartPlatform2.MainGUI;
import RotaryStewartPlatform2.PrimitiveSolids;

import javax.media.opengl.GL2;
import javax.swing.JMenu;
import javax.swing.JMenuItem;


public class World
implements ActionListener {
	/* menus */
	JMenuItem buttonRescan, buttonDisconnect;
	
	/* world contents */
	Camera camera = new Camera();
	RotaryStewartPlatform robot0 = new RotaryStewartPlatform("0");
	
	final int NUM_ROBOTS = 1;
	protected int activeRobot=0;
	
	
	public World() {
		//robot0.MoveBase(new Vector3f(-25f,0f,0f));
		robot0.FinalizeMove();
		/*
		robot1.MoveBase(new Vector3f(25f,0f,0f));
		robot1.RotateBase(180f,0f);
		robot1.FinalizeMove();
		*/
	}
	

    protected void setup( GL2 gl2 ) {
		gl2.glDepthFunc(GL2.GL_LESS);
		gl2.glEnable(GL2.GL_DEPTH_TEST);
		gl2.glDepthMask(true);
    }
    

    public void mouseClicked(MouseEvent e) {
    	
    }
    public void mouseDragged(MouseEvent e) {
    	camera.mouseDragged(e);
    }
    public void mouseEntered(MouseEvent e) {
    	
    }
    public void mouseExited(MouseEvent e) {
    	
    }
    public void mouseMoved(MouseEvent e) {

    }
    public void mousePressed(MouseEvent e) {
    	camera.mousePressed(e);
    	
    }
    public void mouseReleased(MouseEvent e) {
    	camera.mouseReleased(e);
    	
    }
    public void mouseWheelMoved(MouseEvent e) {
    	
    }
    
    public void keyPressed(KeyEvent e) {
    	if(e.getKeyCode() == KeyEvent.VK_SPACE) {
    		activeRobot=(activeRobot+1)%NUM_ROBOTS;
    	}
    	camera.keyPressed(e);
    	switch(activeRobot) {
    	case 0:    	robot0.keyPressed(e); break;
    	//case 1:    	robot1.keyPressed(e); break;
    	}
    }
    
    public void keyReleased(KeyEvent e) {
    	camera.keyReleased(e);
    	switch(activeRobot) {
    	case 0:    	robot0.keyReleased(e); break;
    	//case 1:    	robot1.keyReleased(e); break;
    	}
    }
    

	public void actionPerformed(ActionEvent e) {
		Object subject = e.getSource();
		if(subject==buttonRescan) {
			robot0.DetectSerialPorts();
			//robot1.arduino.DetectSerialPorts();
			//TODO tell RobotTrainer to update all menus
			MainGUI.getSingleton().updateMenu();
			return;
		}
		if(subject==buttonDisconnect) {
			robot0.ClosePort();
			//robot1.arduino.ClosePort();
			MainGUI.getSingleton().updateMenu();
			return;
		}
	}
	
    public JMenu updateMenu() {
    	JMenu menu, subMenu;
        
        // connection menu
        menu = new JMenu("Connection(s)");
        menu.setMnemonic(KeyEvent.VK_T);
        menu.getAccessibleContext().setAccessibleDescription("Connection settings.");
        
    	subMenu=robot0.getMenu();
        subMenu.setText("Arm 0");
        menu.add(subMenu);
/*
     	subMenu=robot1.getMenu();
        subMenu.setText("Arm 1");
        menu.add(subMenu);
*/
        buttonRescan = new JMenuItem("Rescan Ports",KeyEvent.VK_R);
        buttonRescan.getAccessibleContext().setAccessibleDescription("Rescan the available ports.");
        buttonRescan.addActionListener(this);
        menu.add(buttonRescan);

        menu.addSeparator();
        
        buttonDisconnect = new JMenuItem("Disconnect",KeyEvent.VK_D);
        buttonDisconnect.addActionListener(this);
        menu.add(buttonDisconnect);
        
        return menu;
    }

	protected double t = 0.0;
	
	public void render(GL2 gl2, float dt ) {
		//gl2.glEnable(GL2.GL_CULL_FACE);
		
		gl2.glPushMatrix();
			 // Enable lighting
			gl2.glEnable(GL2.GL_LIGHTING);
			gl2.glEnable(GL2.GL_LIGHT0);
			gl2.glEnable(GL2.GL_LIGHT1);
			gl2.glEnable(GL2.GL_COLOR_MATERIAL);
	        //gl2.glColorMaterial( GL2.GL_FRONT_AND_BACK, GL2.GL_AMBIENT_AND_DIFFUSE );
			//gl2.glShadeModel(GL2.GL_SMOOTH);
			//gl2.glEnable(GL2.GL_DEPTH_TEST);
			gl2.glMaterialf(GL2.GL_FRONT, GL2.GL_SHININESS, 100.0f);

			//gl2.glLightfv(GL2.GL_LIGHT0, GL2.GL_EMISSION, new float[] { 0.1f, 0.1f, 0.1f, 1.0f },0);
		    gl2.glLightfv(GL2.GL_LIGHT0, GL2.GL_AMBIENT , new float[] { 0.2f, 0.2f, 0.2f, 1.0f },0);
		    gl2.glLightfv(GL2.GL_LIGHT0, GL2.GL_DIFFUSE , new float[] { 0.08f, 0.08f, 0.08f, 1.0f },0);
		    gl2.glLightfv(GL2.GL_LIGHT0, GL2.GL_SPECULAR, new float[] { 0.02f, 0.02f, 0.02f, 1.0f },0);
		    
		    gl2.glLightfv(GL2.GL_LIGHT1, GL2.GL_DIFFUSE , new float[] { 0.04f, 0.04f, 0.04f, 1.0f },0);
		    gl2.glLightfv(GL2.GL_LIGHT1, GL2.GL_SPECULAR, new float[] { 0.2f, 0.2f, 0.2f, 1.0f },0);
			
			camera.render(gl2);


			//t+=dt;
			float x=100.0f*(float)Math.cos(t);
			float y=100.0f*(float)Math.sin(t);
			float z=75.0f*(float)Math.sin(t*0.5);
			
			gl2.glLightfv(GL2.GL_LIGHT0, GL2.GL_POSITION, new float[]{x,y,z,1.0f},0);
			gl2.glLightfv(GL2.GL_LIGHT1, GL2.GL_POSITION, new float[]{-x,-y,-z,1.0f},0);


			gl2.glDisable(GL2.GL_LIGHTING);
			PrimitiveSolids.drawGrid(gl2);
			gl2.glEnable(GL2.GL_LIGHTING);

			robot0.PrepareMove(dt);
			//robot1.PrepareMove(dt);
			//if(WillCollide(robot0,robot1) == false) 
			{
				robot0.FinalizeMove();
				//robot1.FinalizeMove();
			}

			robot0.render(gl2);
			//robot1.render(gl2);
			
		gl2.glPopMatrix();
	}

	boolean WillCollide(RotaryStewartPlatform a,RotaryStewartPlatform b) {
		// TODO complete me
		//Get the cylinders for each robot
		BoundingVolume [] from = a.GetBoundingVolumes();
		BoundingVolume [] to = b.GetBoundingVolumes();
		// test cylinder/cylinder intersection
		for(int i=0;i<from.length;++i) {
			for(int j=0;j<to.length;++j) {
				if(IntersectionTester.CylinderCylinder((Cylinder)from[i],(Cylinder)to[i])) {
					return true;
				}
			}
		}
		// if there is any hit, return true.
		return false;
	}

	
	public void Init(GL2 gl2) {
		robot0.Init(gl2);
	}
}

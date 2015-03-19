package RotaryStewartPlatform2;
import javax.vecmath.Vector3f;
import javax.media.opengl.GL2;

import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;

public class Camera {
	/** position of camera */
	Vector3f position = new Vector3f();
	Vector3f forward = new Vector3f(0,1,0);
	Vector3f up = new Vector3f(0,0,1);
	Vector3f right = new Vector3f(1,0,0);
	float pan=0, tilt=0;
	int prevMouseX, prevMouseY;
	boolean mouseRButtonDown = false;
	boolean wDown=false;
	boolean sDown=false;
	boolean aDown=false;
	boolean dDown=false;
	boolean qDown=false;
	boolean eDown=false;

	
	public Camera() {}

	
	public void mousePressed(MouseEvent e) {
        prevMouseX = e.getX();
        prevMouseY = e.getY();
        if ((e.getModifiers() & MouseEvent.BUTTON1_MASK) != 0) {
        	mouseRButtonDown = true;
        }
	}
	
	
	public void mouseReleased(MouseEvent e) {
        if ((e.getModifiers() & MouseEvent.BUTTON1_MASK) != 0) {
        	mouseRButtonDown = false;
        }
	}
	
	
	public void mouseDragged(MouseEvent e) {
		if (mouseRButtonDown==true) {
	        int x = e.getX();
	        int y = e.getY();
			pan  += x - prevMouseX;
			tilt -= y - prevMouseY;
			prevMouseX = x;
			prevMouseY = y;
			if(tilt < 1) tilt=1;
			if(tilt > 179) tilt= 179;
		}
	}
	
	
	public void keyPressed(KeyEvent e) {
        int kc = e.getKeyCode();
        switch(kc) {
        case KeyEvent.VK_W: wDown=true;  break;
        case KeyEvent.VK_A: aDown=true;  break;
        case KeyEvent.VK_S: sDown=true;  break;
        case KeyEvent.VK_D: dDown=true;  break;
        case KeyEvent.VK_Q: qDown=true;  break;
        case KeyEvent.VK_E: eDown=true;  break;
        }
	}
	
	
	public void keyReleased(KeyEvent e) {
        int kc = e.getKeyCode();
        switch(kc) {
        case KeyEvent.VK_W: wDown=false;  break;
        case KeyEvent.VK_A: aDown=false;  break;
        case KeyEvent.VK_S: sDown=false;  break;
        case KeyEvent.VK_D: dDown=false;  break;
        case KeyEvent.VK_Q: qDown=false;  break;
        case KeyEvent.VK_E: eDown=false;  break;
        }
	}
	
	
	void render(GL2 gl2) {
		// calculate new vectors for translation based on pan/tilt angles
		forward.y = (float)Math.sin((-pan-90) * Math.PI/180.0) * (float)Math.cos((90-tilt) * Math.PI/180.0);
		forward.x = (float)Math.cos((-pan-90) * Math.PI/180.0) * (float)Math.cos((90-tilt) * Math.PI/180.0);
		forward.z =                                              (float)Math.sin((90-tilt) * Math.PI/180.0);
		
		up.set(0,0,1);

		right.cross(forward, up);
		right.normalize();
		up.cross(right, forward);
		up.normalize();
		
		
		// move the camera
		Vector3f temp = new Vector3f();
		Vector3f direction = new Vector3f(0,0,0);
		float vel = 0.75f;
		boolean changed = false;
		
		// which way do we want to move?
		float delta=1;
		if(wDown != sDown) {
			if (wDown) {
				// forward
				temp.set(forward);
				temp.scale(delta);
				direction.add(temp);
				changed = true;
			}
			if (sDown) {
				// back
				temp.set(forward);
				temp.scale(-delta);
				direction.add(temp);
				changed = true;
			}
		}
		if(aDown != dDown) {
			if (aDown) {
				// strafe left
				temp.set(right);
				temp.scale(-delta);
				direction.add(temp);
				changed = true;
			}
			if (dDown) {
				// strafe right
				temp.set(right);
				temp.scale(delta);
				direction.add(temp);
				changed = true;
			}
		}
		if(qDown != eDown) {
			if (qDown) {
				// strafe left
				temp.set(up);
				temp.scale(delta);
				direction.add(temp);
				changed = true;
			}
			if (eDown) {
				// strafe right
				temp.set(up);
				temp.scale(-delta);
				direction.add(temp);
				changed = true;
			}
		}
		
		if(changed) {
			direction.normalize();
			direction.scale(vel);
			position.add(direction);
		}
		
		// move camera
		gl2.glRotatef(tilt, -1, 0, 0);
		gl2.glRotatef(pan,0,0,1);
		gl2.glTranslatef(position.x,position.y,position.z);
	}
}

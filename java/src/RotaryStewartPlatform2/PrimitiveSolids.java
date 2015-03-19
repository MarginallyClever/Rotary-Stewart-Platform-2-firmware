package RotaryStewartPlatform2;
import javax.vecmath.Vector3f;
import javax.media.opengl.GL2;


public class PrimitiveSolids {
	static void drawCylinder(GL2 gl2,Cylinder tube) {		
		//gl2.glBegin(GL2.GL_LINES);
		//gl2.glVertex3f(tube.GetP1().x, tube.GetP1().y, tube.GetP1().z);
		//gl2.glVertex3f(tube.GetP2().x, tube.GetP2().y, tube.GetP2().z);
		//gl2.glEnd();

		Vector3f tx = new Vector3f();
		Vector3f ty = new Vector3f();
		Vector3f t1 = new Vector3f();
		Vector3f t2 = new Vector3f();
		Vector3f n = new Vector3f();
		
		int i;
		int c=10;
		
		// left
		gl2.glBegin(GL2.GL_TRIANGLE_FAN);
		gl2.glNormal3f(-tube.GetN().x,-tube.GetN().y,-tube.GetN().z);
		for(i=0;i<=c;++i) {
			tx.set(tube.GetR());
			ty.set(tube.GetF());

			float ratio= (float)Math.PI * 2.0f * (float)i/(float)c;
			tx.scale((float)Math.sin(ratio)*tube.radius);
			ty.scale((float)Math.cos(ratio)*tube.radius);
			t1.set(tube.GetP1());
			t1.add(tx);
			t1.add(ty);
			gl2.glVertex3f(t1.x,t1.y,t1.z);
		}
		gl2.glEnd();
		// right
		gl2.glBegin(GL2.GL_TRIANGLE_FAN);
		gl2.glNormal3f(tube.GetN().x,tube.GetN().y,tube.GetN().z);
		for(i=0;i<=c;++i) {
			tx.set(tube.GetR());
			ty.set(tube.GetF());

			float ratio= (float)Math.PI * 2.0f * (float)i/(float)c;
			tx.scale((float)Math.sin(ratio)*tube.radius);
			ty.scale((float)Math.cos(ratio)*tube.radius);
			t1.set(tube.GetP2());
			t1.add(tx);
			t1.add(ty);
			gl2.glVertex3f(t1.x,t1.y,t1.z);
		}
		gl2.glEnd();

		// edge
		gl2.glBegin(GL2.GL_TRIANGLE_STRIP);
		for(i=0;i<=c;++i) {
			tx.set(tube.GetR());
			ty.set(tube.GetF());

			float ratio= (float)Math.PI * 2.0f * (float)i/(float)c;
			tx.scale((float)Math.sin(ratio)*tube.radius);
			ty.scale((float)Math.cos(ratio)*tube.radius);
			t1.set(tube.GetP1());
			t1.add(tx);
			t1.add(ty);
			
			t2.set(tx);
			t2.add(ty);
			n.set(t2);
			n.normalize();
			gl2.glNormal3f(n.x,n.y,n.z);
			t2.add(tube.GetP2());
			gl2.glVertex3f(t1.x,t1.y,t1.z);
			gl2.glVertex3f(t2.x,t2.y,t2.z);
			
		}
		gl2.glEnd();
	}

	// TODO: move this to Cylinder?
	static void drawCylinder(GL2 gl2,float thickness,float radius) {
		int i;
		int c=36;
		
		// left
		gl2.glBegin(GL2.GL_TRIANGLE_FAN);
		gl2.glNormal3f(0,1,0);
		for(i=0;i<=c;++i) {
			float ratio= (float)Math.PI * 2.0f * (float)i/(float)c;
			gl2.glVertex3f((float)Math.sin(ratio)*radius,
							thickness,
							(float)Math.cos(ratio)*radius);
		}
		gl2.glEnd();
		// right
		gl2.glBegin(GL2.GL_TRIANGLE_FAN);
		gl2.glNormal3f(0,-1,0);
		for(i=0;i<=c;++i) {
			float ratio= (float)Math.PI * 2.0f * (float)i/(float)c;
			gl2.glVertex3f((float)Math.sin(ratio)*radius,
							-thickness,
							(float)Math.cos(ratio)*radius);
		}
		gl2.glEnd();

		// edge
		gl2.glBegin(GL2.GL_TRIANGLE_STRIP);
		for(i=0;i<=c;++i) {
			float ratio= (float)Math.PI * 2.0f * (float)i/(float)c;
			float a=(float)Math.sin(ratio)*radius;
			float b=thickness;
			float d=(float)Math.cos(ratio)*radius;
			gl2.glNormal3f(a,0,d);
			gl2.glVertex3f(a,b,d);
			gl2.glVertex3f(a,-b,d);
		}
		gl2.glEnd();
	}
	
	static void drawBox(GL2 gl2,float depth,float width,float height) {
		width/=2;
		depth/=2;

		gl2.glPushMatrix();
		gl2.glBegin(GL2.GL_QUADS);
		// bottom
		gl2.glNormal3f( 0, 0,-1);
		gl2.glVertex3f(-width,-depth,0);
		gl2.glVertex3f( width,-depth,0);
		gl2.glVertex3f( width, depth,0);
		gl2.glVertex3f(-width, depth,0);

		// top
		gl2.glNormal3f( 0, 0, 1);
		gl2.glVertex3f( width, depth,height);
		gl2.glVertex3f(-width, depth,height);
		gl2.glVertex3f(-width,-depth,height);
		gl2.glVertex3f( width,-depth,height);

		
		// side
		gl2.glNormal3f( 0, 1, 0);
		gl2.glVertex3f(-width, depth,height);
		gl2.glVertex3f( width, depth,height);
		gl2.glVertex3f( width, depth,0);
		gl2.glVertex3f(-width, depth,0);
		
		gl2.glNormal3f( 0,-1, 0);
		gl2.glVertex3f( width,-depth,height);
		gl2.glVertex3f(-width,-depth,height);
		gl2.glVertex3f(-width,-depth,0);
		gl2.glVertex3f( width,-depth,0);

		gl2.glNormal3f( 1, 0, 0);
		gl2.glVertex3f( width,-depth,0);
		gl2.glVertex3f( width,-depth,height);
		gl2.glVertex3f( width, depth,height);
		gl2.glVertex3f( width, depth,0);
	
		gl2.glNormal3f(-1, 0, 0);
		gl2.glVertex3f(-width, depth,height);
		gl2.glVertex3f(-width,-depth,height);
		gl2.glVertex3f(-width,-depth,0);
		gl2.glVertex3f(-width, depth,0);

		gl2.glEnd();
		
		gl2.glPopMatrix();
	}
	

	static void drawStar(GL2 gl2,Vector3f p) {
		drawStar(gl2,p,1.0f);
	}
	
	
	static void drawStar(GL2 gl2,Vector3f p,float size) {
		size/=2.0f;
		gl2.glPushMatrix();
		gl2.glTranslatef(p.x, p.y, p.z);
		gl2.glBegin(GL2.GL_LINES);
		gl2.glVertex3f(-size, 0, 0);
		gl2.glVertex3f(size, 0, 0);
		gl2.glVertex3f(0, -size, 0);
		gl2.glVertex3f(0, size, 0);
		gl2.glVertex3f(0, 0, -size);
		gl2.glVertex3f(0, 0, size);
		gl2.glEnd();
		gl2.glPopMatrix();
	}

	
	static public void drawGrid(GL2 gl2,int grid_size,int grid_space) {
		gl2.glNormal3f(0,0,1);
	
		gl2.glBegin(GL2.GL_LINES);
		for(int i=-grid_size;i<=grid_size;i+=grid_space) {
			for(int j=-grid_size;j<=grid_size;j+=grid_space) {
				gl2.glVertex3f(i,-grid_size,0);
				gl2.glVertex3f(i,grid_size,0);					
				gl2.glVertex3f(-grid_size,j,0);
				gl2.glVertex3f(grid_size,j,0);					
			}
		}
		gl2.glEnd();
	}
	
	static public void DrawXYZ(GL2 gl2) {
		gl2.glBegin(GL2.GL_LINES);
		
		// +X line
		gl2.glColor3f(1,0,0);
		gl2.glVertex3f(0,0,0);
		gl2.glVertex3f(1,0,0);
		// +Y line
		gl2.glColor3f(0,1,0);
		gl2.glVertex3f(0,0,0);
		gl2.glVertex3f(0,1,0);
		// +Z line
		gl2.glColor3f(0,0,1);
		gl2.glVertex3f(0,0,0);
		gl2.glVertex3f(0,0,1);
	
		gl2.glEnd();
	}
	
	static public void drawGrid(GL2 gl2) {
		drawGrid(gl2,50,1);
	}
	
	
	protected void drawMatrix(GL2 gl2,Vector3f p,Vector3f u,Vector3f v,Vector3f w) {
		drawMatrix(gl2,p,u,v,w,1);
	}
	
	protected void drawMatrix(GL2 gl2,Vector3f p,Vector3f u,Vector3f v,Vector3f w,float scale) {
		gl2.glPushMatrix();
		gl2.glDisable(GL2.GL_DEPTH_TEST);
		gl2.glTranslatef(p.x, p.y, p.z);
		gl2.glScalef(scale, scale, scale);
		
		gl2.glBegin(GL2.GL_LINES);
		gl2.glColor3f(1,1,0);		gl2.glVertex3f(0,0,0);		gl2.glVertex3f(u.x,u.y,u.z);
		gl2.glColor3f(0,1,1);		gl2.glVertex3f(0,0,0);		gl2.glVertex3f(v.x,v.y,v.z);
		gl2.glColor3f(1,0,1);		gl2.glVertex3f(0,0,0);		gl2.glVertex3f(w.x,w.y,w.z);
		gl2.glEnd();
		
		gl2.glEnable(GL2.GL_DEPTH_TEST);
		gl2.glPopMatrix();
	}
}

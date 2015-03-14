package RotaryStewartPlatform2;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.FloatBuffer;

import javax.media.opengl.GL2;


public class Model {
	static int NUM_BUFFERS=2;  // verts, normals
	
	String name;
	int num_triangles;

	int VBO[] = null;

	// much help from http://www.java-gaming.org/index.php?;topic=18710.0
	void Load(GL2 gl2,String fname) {
		name=fname;

		BufferedReader br = null;
		try {
		    br = new BufferedReader(new FileReader(new File(fname)));

			String line;
			int j=0;
			while( ( line = br.readLine() ) != null ) {
				if( line.length() < 16 ) continue;
				j++;
				if(j==4) {
					j=0;
					num_triangles++;
				}
			}

			VBO = new int[NUM_BUFFERS];
			gl2.glGenBuffers(NUM_BUFFERS, VBO, 0);  // 2 = one for vertexes, one for normals
			int totalBufferSize = num_triangles*3*3;
			FloatBuffer vertices = FloatBuffer.allocate(totalBufferSize);  // num_triangles * points per triangle * float per point
			FloatBuffer normals = FloatBuffer.allocate(totalBufferSize);  // num_triangles * normal per triangle (1) * float per point

		    br.close();
		    br = new BufferedReader(new FileReader(new File(fname)));
			j=0;
			long a=0, b=0;
			float x=0,y=0,z=0;
			while( ( line = br.readLine() ) != null ) {
				if( line.length() < 16 ) continue;
				String c[] = line.trim().split(" ");
				if(c[0].equals("solid")) continue;
				if(c[0].equals("outer")) continue;
				if(c[0].equals("endloop")) continue;
				if(c[0].equals("endfacet")) continue;
				if(c[0].equals("facet")) {
					x=Float.parseFloat(c[2]);
					y=Float.parseFloat(c[3]);
					z=Float.parseFloat(c[4]);
				} else if(c[0].equals("vertex")) {
					x=Float.parseFloat(c[1]);
					y=Float.parseFloat(c[2]);
					z=Float.parseFloat(c[3]);
				} else {
					x=Float.parseFloat(c[0]);
					y=Float.parseFloat(c[1]);
					z=Float.parseFloat(c[2]);
				}
				if(j==0) {
					++a;
					normals.put(x);
					normals.put(y);
					normals.put(z);
					
					normals.put(x);
					normals.put(y);
					normals.put(z);
					
					normals.put(x);
					normals.put(y);
					normals.put(z);
				} else {
					++b;
					vertices.put(x);
					vertices.put(y);
					vertices.put(z);
				}
				j = (j+1)%4;
			}
			
			assert(a==b);
			
			int s=(Float.SIZE/8);  // bits per float / bits per byte = bytes per float
			
			// bind a buffer
			vertices.rewind();
			gl2.glBindBuffer(GL2.GL_ARRAY_BUFFER, VBO[0]);
		    // Write out vertex buffer to the currently bound VBO.
		    gl2.glBufferData(GL2.GL_ARRAY_BUFFER, totalBufferSize*s, vertices, GL2.GL_STATIC_DRAW);
		    
		    // repeat for normals
			normals.rewind();
			gl2.glBindBuffer(GL2.GL_ARRAY_BUFFER, VBO[1]);
		    gl2.glBufferData(GL2.GL_ARRAY_BUFFER, totalBufferSize*s, normals, GL2.GL_STATIC_DRAW);
		}
		catch(IOException e) {
			e.printStackTrace();
		}
		finally {
			try {
				if(br != null) br.close();
			}
			catch(IOException e) {
				e.printStackTrace();
			}
		}
	}
	
	void Draw(GL2 gl2) {
		if(VBO==null) return;
		
		gl2.glEnableClientState(GL2.GL_VERTEX_ARRAY);
		gl2.glEnableClientState(GL2.GL_NORMAL_ARRAY);
		//gl2.glColor3f(1.0f, 1.0f, 1.0f);
	      
		// Bind the normal buffer to work with
		gl2.glBindBuffer(GL2.GL_ARRAY_BUFFER, VBO[1]);
		gl2.glNormalPointer(GL2.GL_FLOAT, 0, 0);

		// Bind the vertex buffer to work with
		gl2.glBindBuffer(GL2.GL_ARRAY_BUFFER, VBO[0]);
		gl2.glVertexPointer(3, GL2.GL_FLOAT, 0, 0);
  
		gl2.glDrawArrays(GL2.GL_TRIANGLES, 0, num_triangles*3);
		//gl2.glDrawArrays(GL2.GL_LINE_LOOP, 0, num_triangles*3);
		gl2.glDisableClientState(GL2.GL_VERTEX_ARRAY);
		gl2.glDisableClientState(GL2.GL_NORMAL_ARRAY);
	}
}

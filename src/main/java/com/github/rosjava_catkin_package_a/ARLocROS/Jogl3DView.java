package com.github.rosjava_catkin_package_a.ARLocROS;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point3;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLCapabilities;
import com.jogamp.opengl.GLEventListener;
import com.jogamp.opengl.awt.GLJPanel;
import com.jogamp.opengl.glu.GLU;

/**
 * A template for a basic JOGL application with support for animation and for
 * keyboard and mouse event handling. To enable the support, uncomment the
 * appropriate lines in the init() method. As an example, the program draws the
 * GLUT teapot.
 */
public class Jogl3DView extends JPanel
		implements GLEventListener, KeyListener, MouseListener, MouseMotionListener, ActionListener {

	private static final int dispHeight = 480;
	private static final int dispWidth = 640;
	/**
	 * 
	 */
	private static final long serialVersionUID = -8553420347862429645L;

	private static Jogl3DView jt;
	private Mat rvec;
	private MatOfDouble tvec;

	private GLJPanel display;
	public Jogl3DView() {
		GLCapabilities caps = new GLCapabilities(null);
		display = new GLJPanel(caps);
		display.setPreferredSize(new Dimension(dispWidth, dispHeight)); // TODO:
																		// set
																		// display
		// patterntSize here
		display.addGLEventListener(this);
		setLayout(new BorderLayout());
		add(display, BorderLayout.CENTER);
		// TODO: Other components could be added to the main panel.

		// TODO: Uncomment the next two lines to enable keyboard event handling
		display.requestFocusInWindow();
		display.addKeyListener(this);

		// TODO: Uncomment the next one or two lines to enable mouse event
		// handling
		display.addMouseListener(this);
		display.addMouseMotionListener(this);

	}

	// --------------- Methods of the GLEventListener interface -----------

	/**
	 * This method is called when the OpenGL display needs to be redrawn.
	 */

	public void display(GLAutoDrawable drawable) {
		// called when the panel needs to be drawn
		GLU glu = new GLU();
		GL2 gl = drawable.getGL().getGL2();
		gl.glClearColor(0, 0, 0, 0);
		gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);

		gl.glMatrixMode(GL2.GL_PROJECTION); // TODO: Set up a better projection?
		gl.glLoadIdentity();

		// Perspective.
		float widthHeightRatio = (float) dispWidth / (float) dispHeight;
		glu.gluPerspective(45, widthHeightRatio, 1, 1000);

		Mat R = new Mat(3, 3, CvType.CV_32FC1);
		Mat einser = new MatOfDouble(0.0, 0.0, 1.0);
		Calib3d.Rodrigues(rvec, R);
		R = R.t();
		Core.gemm(R, einser, 1, new Mat(), 0, einser, 0);
		Core.add(tvec, einser, einser);
		// System.out.println(einser.dump());
		glu.gluLookAt(tvec.get(0, 0)[0], tvec.get(1, 0)[0], tvec.get(2, 0)[0], einser.get(0, 0)[0], einser.get(1, 0)[0],
				einser.get(2, 0)[0], 0, 0,1);

		gl.glMatrixMode(GL2.GL_MODELVIEW);

		gl.glLoadIdentity(); // Set up modelview transform.

		gl.glColor3ub((byte) 255, (byte) 255, (byte) 255);

		gl.glBegin(GL2.GL_POINTS);

		List<Point3> points3dlist = null;//MarkerConfig.getUnordered3DPointList();
		boolean first = true;
		for (Point3 fp : points3dlist) {
			if (first) {
				gl.glColor3ub((byte) 255, (byte) 255, (byte) 255);
				first=false;
			} else
				gl.glColor3ub((byte) 255, (byte) 0, (byte) 0);
			gl.glVertex3d(fp.x, fp.y, fp.z);
		}
		gl.glEnd();
		//draw axis
		// save previous matrix
		gl.glPushMatrix();
		// clear matrix
		gl.glLoadIdentity();
		// apply rotations
//		gl.glRotate3f(rotX, 1.0, 0.0, 0.0);
//		gl.glRotate3f(rotY, 0.0, 1.0, 0.0);
//		gl.glRotate3f(rotZ, 0.0, 0.0, 1.0);
		// move the axes to the screen corner
		//gl.glTranslated(-3.0, -2.0, 0.0);
		// draw our axes
		gl.glBegin(GL2.GL_LINES);
		// draw line for x axis
		gl.glColor3d(1.0, 0.0, 0.0);
		gl.glVertex3d(0.0, 0.0, 0.0);
		gl.glVertex3d(1.0, 0.0, 0.0);
		// draw line for y axis
		gl.glColor3d(0.0, 1.0, 0.0);
		gl.glVertex3d(0.0, 0.0, 0.0);
		gl.glVertex3d(0.0, 1.0, 0.0);
		// draw line for Z axis
		gl.glColor3d(0.0, 0.0, 1.0);
		gl.glVertex3d(0.0, 0.0, 0.0);
		gl.glVertex3d(0.0, 0.0, 1.0);
		gl.glEnd();
		// load the previous matrix
		gl.glPopMatrix();

		

	}

	
	/**
	 * This is called when the GLJPanel is first created. It can be used to
	 * initialize the OpenGL drawing context.
	 */

	public void init(GLAutoDrawable drawable) {
		// called when the panel is created
		GL2 gl = drawable.getGL().getGL2();
		gl.glClearColor(0.8F, 0.8F, 0.8F, 1.0F);
		gl.glEnable(GL.GL_DEPTH_TEST);
		gl.glEnable(GL2.GL_LIGHTING);
		gl.glEnable(GL2.GL_LIGHT0);
		gl.glEnable(GL2.GL_COLOR_MATERIAL);

	}

	public static Jogl3DView start() {
		if (jt == null) {

			// getting the capabilities object of GL2 profile
			JFrame window = new JFrame("JOGL");
			jt = new Jogl3DView();
			jt.rvec = new Mat(3, 1, CvType.CV_64F);
			jt.tvec = new MatOfDouble(1.0, 1.0, 1.0);
			window.setContentPane(jt);
			window.pack();
			window.setLocation(50, 50);
			window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			window.setVisible(true);
		}
		return jt;

	}

	public void showandsafe(Mat rvec2, MatOfDouble tvec2) {
		rvec = rvec2;
		tvec = tvec2;
		display.repaint();

	}

	@Override
	public void mouseClicked(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseEntered(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseExited(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseMoved(MouseEvent e) {
		// TODO Auto-generated method stub

	}

	@Override
	public void actionPerformed(ActionEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseDragged(MouseEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mousePressed(MouseEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseReleased(MouseEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void keyPressed(KeyEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void keyReleased(KeyEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void keyTyped(KeyEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void dispose(GLAutoDrawable drawable) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
		// TODO Auto-generated method stub
		
	}

}
import java.awt.Color;

import javax.media.j3d.Appearance;
import javax.media.j3d.BranchGroup;
import javax.media.j3d.ColoringAttributes;
import javax.media.j3d.GeometryArray;
import javax.media.j3d.Node;
import javax.media.j3d.PointArray;
import javax.media.j3d.PointAttributes;
import javax.media.j3d.PolygonAttributes;
import javax.media.j3d.Shape3D;
import javax.media.j3d.TriangleStripArray;
import javax.vecmath.Color3f;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple2f;


public class Square extends PhysicsObject {
	public float length;
	
	public Square(float mass, float positionX, float positionY, float velocityX, float velocityY, 
			float orientation, float angularVelocity, float length, Color3f color1, Color3f color2) {
		super(mass, positionX, positionY, velocityX, velocityY, orientation,
				angularVelocity);
		if (length <= 0)
			throw new IllegalArgumentException();
		
		momentOfInertia = mass * length * length / 4;
		centerOfMass.x = length / 4;
		centerOfMass.y = length / 4;
		// Using the parallel axis theorem
//		momentOfInertia += mass * centerOfMass.lengthSquared();
		this.length = length;
		TG.addChild(createShape(length, color1, color2));
	}

	public Square(float mass, Tuple2f position, Tuple2f velocity,
			float orientation, float angularVelocity) {
		super(mass, position, velocity, orientation, angularVelocity);
		// TODO Auto-generated constructor stub
	}
	
	private Node createShape(float radius, Color3f color1, Color3f color2) {
		
		TriangleStripArray topGeometry = new TriangleStripArray(6, GeometryArray.COORDINATES, new int[] {4});
		Point3f[] vertices = new Point3f[4];
		vertices[0] = new Point3f(-radius, radius, 0f);
		vertices[1] = new Point3f(-radius, -radius, 0f);
		vertices[2] = new Point3f(radius, radius, 0f);
		vertices[3] = new Point3f(radius, -radius, 0f);
		
		topGeometry.setCoordinates(0, vertices);
		
		PointArray centerOfMassGeometry = new PointArray(1, GeometryArray.COORDINATES);
		centerOfMassGeometry.setCoordinate(0, new Point3f(centerOfMass.x, centerOfMass.y, 0));
		
		BranchGroup root = new BranchGroup();
		if (color1 == null)
			color1 = new Color3f(Color.getHSBColor((float)Math.random(), 1, 1));
		Appearance appearance = new Appearance();
		appearance.setColoringAttributes(new ColoringAttributes(color1, ColoringAttributes.FASTEST));
		PolygonAttributes polyAttr = new PolygonAttributes(PolygonAttributes.POLYGON_FILL, PolygonAttributes.CULL_NONE, 0);
		appearance.setPolygonAttributes(polyAttr);
		root.addChild(new Shape3D(topGeometry, appearance));
		
		if (color2 == null)
			color2 = new Color3f(Color.getHSBColor((float)Math.random(), 1, 1));
		appearance = new Appearance();
		appearance.setColoringAttributes(new ColoringAttributes(color2, ColoringAttributes.FASTEST));
		appearance.setPolygonAttributes(polyAttr);
		
		appearance = new Appearance();
		appearance.setPointAttributes(new PointAttributes(4, true));
		root.addChild(new Shape3D(centerOfMassGeometry, appearance));
	
		return root;
	}
}

import java.awt.*;
import javax.media.j3d.*;
import javax.vecmath.*;

// Isosceles right triangle
public class Triangle extends ConvexPolygon {

	private Vector2f[] vertexCache;
	private Vector2f[] normalCache;
	
	public Triangle(float mass, float positionX, float positionY, float velocityX, float velocityY, float orientation, float angularVelocity, float width, Color3f color) {
		super(mass, positionX, positionY, velocityX, velocityY, orientation, angularVelocity);
		
		if (width <= 0)
			throw new IllegalArgumentException();
		
		// True center of mass for an isosceles right triangle
		centerOfMass.x = centerOfMass.y = width / 3;
        momentOfInertia = (float)(Math.pow(width, 2) / 9);
        VERTICES = new float[]{0, 0, width, 0, 0, width};
		TG.addChild(createShape(color));
	}
	
	private Node createShape(Color3f color) {
		TriangleArray geometry = new TriangleArray(3, GeometryArray.COORDINATES);
		for (int i = 0; i < VERTICES.length; i += 2)
			geometry.setCoordinate(i / 2, new Point3f(VERTICES[i], VERTICES[i+1], 0));

		PointArray centerOfMassGeometry = new PointArray(1, GeometryArray.COORDINATES);
		centerOfMassGeometry.setCoordinate(0, new Point3f(centerOfMass.x, centerOfMass.y, 0));
		
		BranchGroup root = new BranchGroup();
		if (color == null)
			color = new Color3f(Color.getHSBColor((float)Math.random(), 1, 1));
		Appearance appearance = new Appearance();
		appearance.setColoringAttributes(new ColoringAttributes(color, ColoringAttributes.FASTEST));
		PolygonAttributes polyAttr = new PolygonAttributes(PolygonAttributes.POLYGON_FILL, PolygonAttributes.CULL_NONE, 0);
		appearance.setPolygonAttributes(polyAttr);
		root.addChild(new Shape3D(geometry, appearance));
		
		appearance = new Appearance();
		appearance.setPointAttributes(new PointAttributes(4, true));
		root.addChild(new Shape3D(centerOfMassGeometry, appearance));
	
		return root;
	}
}

import javax.media.j3d.*;
import javax.vecmath.Color3f;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple2f;
import javax.vecmath.Vector2f;
import java.awt.*;

// Isosceles right triangle
public class RegularConvexPolygon extends ConvexPolygon {

    public RegularConvexPolygon(int sides , float radius ,
                                float mass,
                                float positionX, float positionY,
                                float velocityX, float velocityY,
                                float orientation, float angularVelocity,
                                Color3f color) {
		super(mass, positionX, positionY, velocityX, velocityY, orientation, angularVelocity);

		if (radius <= 0)
			throw new IllegalArgumentException();

        VERTICES = new float[sides * 2];
        for(int i = 0; i < sides; i++) {
            VERTICES[i*2  ] = radius * (float)Math.cos(i * 2 * Math.PI / sides); //x
            VERTICES[i*2+1] = radius * (float)Math.sin(i * 2 * Math.PI / sides); //y
        }

//		// True center of mass for an isosceles right triangle
//		centerOfMass.x = centerOfMass.y = width / 3;
        centerOfMass.x = centerOfMass.y = 0;
		momentOfInertia = (float)(Math.pow(radius, 2) / 9);
		TG.addChild(createShape(radius, color));
	}

//	public RegularConvexPolygon(float mass, Tuple2f position, Tuple2f velocity, float orientation, float angularVelocity, float width, Color3f color) {
//		this(mass, position.x, position.y, velocity.x, velocity.y, orientation, angularVelocity, width, color);
//	}

    protected Node createShape(float radius, Color3f color) {
        TriangleFanArray geometry = new TriangleFanArray(VERTICES.length/2+2, GeometryArray.COORDINATES, new int[]{VERTICES.length/2+2});
        geometry.setCoordinate(0,new Point3f(0,0,0));
        for (int i = 0; i < VERTICES.length; i += 2)
            geometry.setCoordinate(1+ (i / 2), new Point3f(radius * VERTICES[i], radius * VERTICES[i+1], 0));

        geometry.setCoordinate(VERTICES.length/2+1, new Point3f(radius * VERTICES[0], radius * VERTICES[0+1], 0));

        PointArray centerOfMassGeometry = new PointArray(1, GeometryArray.COORDINATES);
        centerOfMassGeometry.setCoordinate(0, new Point3f(centerOfMass.x, centerOfMass.y, 0));

        BranchGroup root = new BranchGroup();
        if (color == null)
            color = new Color3f(Color.getHSBColor((float) Math.random(), 1, 1));
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

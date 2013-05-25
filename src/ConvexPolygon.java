import java.awt.*;
import javax.media.j3d.*;
import javax.vecmath.*;

// Isosceles right triangle
public abstract class ConvexPolygon extends PhysicsObject {
    protected float[] VERTICES = {0, 0, 1, 0, 0, 1};
	private Vector2f[] vertexCache;
	private Vector2f[] normalCache;

	public ConvexPolygon(float mass,
                         float positionX, float positionY,
                         float velocityX, float velocityY,
                         float orientation, float angularVelocity) {
		super(mass, positionX, positionY, velocityX, velocityY, orientation, angularVelocity);
	}
	
	public void clearCaches() {
		vertexCache = null;
		normalCache = null;
	}

    public Vector2f[] getVertices() {
        if (vertexCache == null) {
            vertexCache = new Vector2f[VERTICES.length / 2];
            for (int i = 0; i < VERTICES.length; i += 2) {
                float tmpX = VERTICES[i];
                float tmpY = VERTICES[i+1];
                vertexCache[i/2] = new Vector2f();
                vertexCache[i/2].x = (float)(Math.cos(orientation) * tmpX - Math.sin(orientation) * tmpY) + position.x;
                vertexCache[i/2].y = (float)(Math.sin(orientation) * tmpX + Math.cos(orientation) * tmpY) + position.y;
            }
        }
        return vertexCache;
    }

    public Vector2f[] getNormals() {
        if (normalCache == null) {
            Vector2f[] vertices = getVertices();
            normalCache = new Vector2f[vertices.length];

            for (int i = 0; i < vertices.length; i++) {
                normalCache[i] = new Vector2f();
                normalCache[i].scaleAdd(-1, vertices[i], vertices[(i+1)%vertices.length]);
                normalCache[i].normalize();
                float tmp = normalCache[i].x;
                normalCache[i].x = normalCache[i].y;
                normalCache[i].y = -tmp;
            }
        }
        return normalCache;
    }
}

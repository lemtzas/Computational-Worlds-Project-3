import javax.vecmath.*;

public class CollisionHandler {
	private static final float COEFFICIENT_OF_RESTITUTION = .9f;
	
	public static void checkAndResolveCollision(PhysicsObject a, PhysicsObject b) {
		CollisionInfo ci = getCollisionInfo(a, b);
		if (ci == null)
			return;

		// Vector from the center of mass of object a to the collision point
		Vector2f r_ap = new Vector2f();
		r_ap.scaleAdd(-1, a.getGlobalCenterOfMass(), ci.position);
		// Vector from the center of mass of object b to the collision point
		Vector2f r_bp = new Vector2f();
		r_bp.scaleAdd(-1, b.getGlobalCenterOfMass(), ci.position);
		// Velocity of object a at the point of collision
		Vector2f v_ap1 = new Vector2f();
		v_ap1.x = a.velocity.x - a.angularVelocity * r_ap.y;
		v_ap1.y = a.velocity.y + a.angularVelocity * r_ap.x;
		// Velocity of object b at the point of collision
		Vector2f v_bp1 = new Vector2f();
		v_bp1.x = b.velocity.x - b.angularVelocity * r_bp.y;
		v_bp1.y = b.velocity.y + b.angularVelocity * r_bp.x;
		// The collision impulse
		Vector2f v_ab1 = new Vector2f();
		v_ab1.scaleAdd(-1, v_bp1, v_ap1);
		float tmpA = r_ap.x * ci.normal.y - r_ap.y * ci.normal.x;
		float tmpB = r_bp.x * ci.normal.y - r_bp.y * ci.normal.x;
		float j = -(1 + COEFFICIENT_OF_RESTITUTION) * v_ab1.dot(ci.normal) / (1 / a.mass + 1 / b.mass + tmpA * tmpA / a.momentOfInertia + tmpB * tmpB / b.momentOfInertia);
		// Update object a's velocity
		a.velocity.scaleAdd(j / a.mass, ci.normal, a.velocity);
		// Update object b's velocity
		b.velocity.scaleAdd(-j / b.mass, ci.normal, b.velocity);
		// Update object a's angular velocity
		a.angularVelocity += j * (r_ap.x * ci.normal.y - r_ap.y * ci.normal.x) / a.momentOfInertia;
		// Update object b's angular velocity
		b.angularVelocity -= j * (r_bp.x * ci.normal.y - r_bp.y * ci.normal.x) / b.momentOfInertia;
		// Remove object overlap
		a.position.scaleAdd(-ci.depth / (a.mass * (1 / a.mass + 1 / b.mass)), ci.normal, a.position);
		b.position.scaleAdd(ci.depth / (b.mass * (1 / a.mass + 1 / b.mass)), ci.normal, b.position);
		
		a.clearCaches();
		b.clearCaches();
	}
	
	private static CollisionInfo getCollisionInfo(PhysicsObject a, PhysicsObject b) {
		if (a == b)
			return null;
		
		CollisionInfo ci = null;
		if (a instanceof HalfSpace) {
			if (b instanceof Circle)
				ci = getCollision((HalfSpace)a, (Circle)b);
			else if (b instanceof Triangle)
				ci = getCollision((HalfSpace)a, (Triangle)b);
		} else if (a instanceof Circle) {
			if (b instanceof Circle)
				ci = getCollision((Circle)a, (Circle)b);
            else if (b instanceof Triangle)
                ci = getCollision((Circle)a, (Triangle)b);
		} else if (a instanceof Triangle) {
			if (b instanceof Triangle)
				ci = getCollision((Triangle)a, (Triangle)b);
		}
		return ci;
	}

	private static CollisionInfo getCollision(HalfSpace a, Circle b) {
		float distance = a.normal.dot(b.position) - a.intercept - b.radius;
		if (distance < 0) {
			CollisionInfo ci = new CollisionInfo();
			ci.normal = a.normal;
			ci.depth = -distance;
			ci.position = new Vector2f();
			ci.position.scaleAdd(-(b.radius - ci.depth), ci.normal, b.position);
			return ci;
		}
		return null;
	}
	
	private static CollisionInfo getCollision(HalfSpace a, Triangle b) {
		Vector2f[] vertices = b.getVertices();
		float[] distances = new float[vertices.length];
		
		for (int i = 0; i < vertices.length; i++)
			distances[i] = a.normal.dot(vertices[i]) - a.intercept;
		
		int minIndex = 0;
		for (int i = 1; i < distances.length; i++)
			if (distances[i] < distances[minIndex])
				minIndex = i;
		if (distances[minIndex] >= 0)
			return null;
		
		CollisionInfo ci = new CollisionInfo();
		ci.depth = -distances[minIndex];
		ci.normal = a.normal;
		ci.position = new Vector2f(vertices[minIndex]);
		ci.position.scaleAdd(ci.depth, ci.normal, ci.position);
		return ci;
	}

    private static CollisionInfo getCollision(Circle a, Circle b) {
        Vector2f n = new Vector2f();
        n.scaleAdd(-1, a.position, b.position);
        float distance = n.length() - a.radius - b.radius;
        if (distance < 0) {
            CollisionInfo ci = new CollisionInfo();
            n.normalize();
            ci.normal = n;
            ci.depth = -distance;
            ci.position = new Vector2f();
            ci.position.scaleAdd(a.radius - ci.depth / 2, ci.normal, a.position);
            return ci;
        }
        return null;
    }

    /** Circle-Triangle Collision **/
    private static CollisionInfo getCollision(Circle a, Triangle b) {
        Vector2f[] normal = b.getNormals();
        Vector2f[] point = b.getVertices();

        //collide with corners
        Vector2f n = new Vector2f();
        for(int i = 0; i < point.length; i++) {
            n.scaleAdd( -1 , a.position , point[i] );
            float distance = n.length() - a.radius;

            if (distance < 0) { //assume this is the only point it's overlapping with
                CollisionInfo ci = new CollisionInfo();
                n.normalize();
                ci.normal = n;
                ci.depth = -distance;
                ci.position = new Vector2f();
                ci.position.scaleAdd(a.radius - ci.depth / 2, ci.normal, a.position);
                return ci;
            }
        }

        //collide with edges
        float[] intercept = new float[3];
        float[] distances = new float[3];
        boolean noCollision = false;
        for(int i = 0; i < 3; i++) {
            intercept[i] = point[i].dot(normal[i]);
            distances[i] = normal[i].dot(a.position) - intercept[i] - a.radius;
            if(distances[i] >= 0)
                noCollision = true;
        }

        //select closest distance
        float minDistance = distances[0];
        int index = 0;
        for(int i = 1; i < distances.length; i++) {
            if(distances[i] < minDistance) {
                index = i;
                minDistance = distances[i];
            }
        }

        if (!noCollision) {
            System.out.println("collision " + minDistance);
            CollisionInfo ci = new CollisionInfo();
            ci.normal = normal[index];
            ci.depth = -minDistance;
            ci.position = new Vector2f();
            ci.position.scaleAdd((a.radius - ci.depth / 2), ci.normal, a.position);
            return ci;
        }
        return null;
    }
	
	private static CollisionInfo getCollision(Triangle a, Triangle b) {
		Vector2f[] verticesA = a.getVertices();
		Vector2f[] normalsA = a.getNormals();
		Vector2f[] verticesB = b.getVertices();
		Vector2f[] normalsB = b.getNormals();
		float[][] distanceFromA = new float[verticesA.length][verticesB.length];
		float[][] distanceFromB = new float[verticesB.length][verticesA.length];
		int[] indexMinDistanceFromA = new int[verticesA.length];
		int[] indexMinDistanceFromB = new int[verticesB.length];
		
		for (int i = 0; i < verticesA.length; i++) {
			for (int j = 0; j < verticesB.length; j++) {
				Vector2f tmp = new Vector2f();
				tmp.scaleAdd(-1, verticesA[i], verticesB[j]);
				distanceFromA[i][j] = tmp.dot(normalsA[i]);
				if (distanceFromA[i][j] < distanceFromA[i][indexMinDistanceFromA[i]])
					indexMinDistanceFromA[i] = j;
			}
			if (distanceFromA[i][indexMinDistanceFromA[i]] >= 0)
				return null;
		}
		for (int i = 0; i < verticesB.length; i++) {
			for (int j = 0; j < verticesA.length; j++) {
				Vector2f tmp = new Vector2f(verticesA[j]);
				tmp.scaleAdd(-1, verticesB[i], verticesA[j]);
				distanceFromB[i][j] = tmp.dot(normalsB[i]);
				if (distanceFromB[i][j] < distanceFromB[i][indexMinDistanceFromB[i]])
					indexMinDistanceFromB[i] = j;
			}
			if (distanceFromB[i][indexMinDistanceFromB[i]] >= 0)
				return null;
		}
		
		int indexMaxDistanceFromA = 0;
		for (int i = 1; i < verticesA.length; i++)
			if (distanceFromA[i][indexMinDistanceFromA[i]] > distanceFromA[indexMaxDistanceFromA][indexMinDistanceFromA[indexMaxDistanceFromA]])
				indexMaxDistanceFromA = i;
		int indexMaxDistanceFromB = 0;
		for (int i = 1; i < verticesB.length; i++)
			if (distanceFromB[i][indexMinDistanceFromB[i]] > distanceFromB[indexMaxDistanceFromB][indexMinDistanceFromB[indexMaxDistanceFromB]])
				indexMaxDistanceFromB = i;
		
		CollisionInfo ci = new CollisionInfo();
		if (distanceFromA[indexMaxDistanceFromA][indexMinDistanceFromA[indexMaxDistanceFromA]] > distanceFromB[indexMaxDistanceFromB][indexMinDistanceFromB[indexMaxDistanceFromB]]) {
			ci.depth = -distanceFromA[indexMaxDistanceFromA][indexMinDistanceFromA[indexMaxDistanceFromA]];
			ci.normal = new Vector2f(normalsA[indexMaxDistanceFromA]);
			ci.position = new Vector2f(verticesB[indexMinDistanceFromA[indexMaxDistanceFromA]]);
			ci.position.scaleAdd(-ci.depth, ci.normal, ci.position);
		} else {
			ci.depth = -distanceFromB[indexMaxDistanceFromB][indexMinDistanceFromB[indexMaxDistanceFromB]];
			ci.normal = new Vector2f(normalsB[indexMaxDistanceFromB]);
			ci.normal.scale(-1);
			ci.position = new Vector2f(verticesA[indexMinDistanceFromB[indexMaxDistanceFromB]]);
//			ci.position.scaleAdd(ci.depth, ci.normal, ci.position);
		}
		return ci;
	}
}

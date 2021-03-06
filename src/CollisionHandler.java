import javax.vecmath.*;

public class CollisionHandler {
    private static final float COEFFICIENT_OF_RESTITUTION = 0.0f;
    private static final float COEFFICIENT_OF_FRICTION = 0.02f;
	
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
		float j = -(1+COEFFICIENT_OF_RESTITUTION) * v_ab1.dot(ci.normal)
                                    /
                (1/a.mass  +  1/b.mass  +  tmpA*tmpA / a.momentOfInertia  +  tmpB*tmpB / b.momentOfInertia);
        //the friction impulse
        Vector2f n2 = new Vector2f(-ci.normal.y,ci.normal.x);
        n2.scale(n2.dot(v_ab1),n2); //point it in the right direction
        n2.normalize(); //this step sometimes produces NaNs, as the vector is 0.0 , 0.0
        float j2 = -(1+COEFFICIENT_OF_RESTITUTION) * v_ab1.dot(n2)
                /
                (1/a.mass  +  1/b.mass  +  tmpA*tmpA / a.momentOfInertia  +  tmpB*tmpB / b.momentOfInertia);


        float friction = -Math.abs(j2 * COEFFICIENT_OF_FRICTION);
		// Update object a's velocity
        a.velocity.scaleAdd(j / a.mass, ci.normal, a.velocity);
		// Update object b's velocity
		b.velocity.scaleAdd(-j / b.mass, ci.normal, b.velocity);
        //apply friction
        if(!Float.isNaN(n2.x) && !Float.isNaN(n2.y)) { //make sure we can actually apply friction (object is moving perpendicular to collision)
            //to velocity
            float a_vel = a.velocity.dot(n2);
            if(Math.abs(a_vel)  < friction / a.mass) { //limit friction delta-velocity to, at most, current velocity perpendicular to collision
                a.velocity.scaleAdd(-a_vel , n2, a.velocity);
            } else {
                a.velocity.scaleAdd(-friction / a.mass , n2, a.velocity);
            }
            float b_vel = b.velocity.dot(n2);
            if(Math.abs(b_vel) < friction / b.mass) {  //limit friction delta-velocity to, at most, current velocity perpendicular to collision
                b.velocity.scaleAdd(-b_vel, n2, b.velocity);
//                System.out.println("a");
            } else {
                b.velocity.scaleAdd(-friction / b.mass , n2, b.velocity);
            }




//            System.out.println(friction / a.mass + " " + friction / b.mass);
//            System.out.println(a_vel + "-" + b_vel);
        }
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
			else if (b instanceof ConvexPolygon)
				ci = getCollision((HalfSpace)a, (ConvexPolygon)b);
		} else if (a instanceof Circle) {
			if (b instanceof Circle)
				ci = getCollision((Circle)a, (Circle)b);
            else if (b instanceof ConvexPolygon)
                ci = getCollision((Circle)a, (ConvexPolygon)b);
		} else if (a instanceof ConvexPolygon) {
			if (b instanceof ConvexPolygon)
				ci = getCollision((ConvexPolygon)a, (ConvexPolygon)b);
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
	
	private static CollisionInfo getCollision(HalfSpace a, ConvexPolygon b) {
        System.out.println(b.getClass().getName());
		Vector2f[] vertices = b.getVertices();
		float[] distances = new float[vertices.length];
		
		for (int i = 0; i < vertices.length; i++)
			distances[i] = a.normal.dot(vertices[i]) - a.intercept;
		
		int minIndex = 0;
		for (int i = 1; i < distances.length; i++)
			if (distances[i] < distances[minIndex])
				minIndex = i;
        System.out.println(distances[minIndex]);
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
    private static CollisionInfo getCollision(Circle a, ConvexPolygon b) {
        Vector2f[] normal = b.getNormals();
        Vector2f[] point = b.getVertices();
        float[] pointDistance = new float[point.length];
        float minPointDistance = Float.POSITIVE_INFINITY;
        int minPointDistanceIndex = 0;

        //corners
        Vector2f[] n = new Vector2f[point.length];
        for(int i = 0; i < point.length; i++) {
            n[i] = new Vector2f();
            n[i].scaleAdd( -1 , a.position , point[i] );
            pointDistance[i] = n[i].length();// - a.radius;

            if (pointDistance[i] < minPointDistance) {
                minPointDistance = pointDistance[i];
                minPointDistanceIndex = i;
            }
        }
         if(minPointDistance < 0) {
            CollisionInfo ci = new CollisionInfo();
            n[minPointDistanceIndex].normalize();
            ci.normal = n[minPointDistanceIndex];
            ci.depth = -minPointDistance;
            ci.position = new Vector2f();
            ci.position.scaleAdd(a.radius - ci.depth, ci.normal, a.position);
            return ci;
        }

        //inside
        boolean inside = true;
        float[] intercept = new float[point.length];
        float[] distances = new float[point.length];
        for(int i = 0; i < normal.length; i++) {
            intercept[i] = normal[i].dot(point[i]);
            distances[i] = normal[i].dot(a.position) - intercept[i];// + a.radius;
            if(distances[i] >= 0)
                inside = false;
        }
        //collision point generation is of questionable correctness
        if(inside) {
            CollisionInfo ci = new CollisionInfo();
            Vector2f t = new Vector2f();
            t.scaleAdd(-1,a.centerOfMass,b.centerOfMass);
            t.normalize();
            ci.normal = t;
            ci.depth = -t.length();
            ci.position = new Vector2f();
            ci.position.scaleAdd(a.radius - ci.depth/2, ci.normal, a.position);
            return ci;
        }

        //edges
        float[] distances2 = new float[point.length];
        float minDist2 = 0;
        int minDist2Index = 0;
        boolean inside2 = true;
        for(int i = 0; i < normal.length; i++) {
            distances2[i] = normal[i].dot(a.position) - intercept[i] - a.radius;
            if(distances2[i] < minDist2 && distances2[i] >= -a.radius) {
                minDist2 = distances2[i];
                minDist2Index = i;
            }
            if(distances2[i] >= 0) {
                inside2 = false;
            }
        }

        if(inside2) {
            CollisionInfo ci = new CollisionInfo();
            ci.normal = normal[minDist2Index];
            ci.normal.normalize();
            ci.depth = minDist2;
            ci.position = new Vector2f();
            ci.position.scaleAdd((a.radius - ci.depth), ci.normal, a.position);
            return ci;
        }

        return null;
    }
	
	private static CollisionInfo getCollision(ConvexPolygon a, ConvexPolygon b) {
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

package com.acmerobotics.roadrunner.ftc;

import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

public class GeometrySamples {
    // sample: vectorHelpers
    public static double distTo(Vector2d a, Vector2d b) {
        return a.minus(b).norm();
    }

    public static double angleBetween(Vector2d a, Vector2d b) {
        double aNorm = a.norm();
        double bNorm = b.norm();
        if (aNorm == 0.0 || bNorm == 0.0) {
            throw new IllegalArgumentException("Angle requires nonzero vectors");
        }
        double cosine = a.div(aNorm).dot(b.div(bNorm));
        return Math.acos(Math.max(-1.0, Math.min(1.0, cosine)));
    }

    public static Vector2d projectOnto(Vector2d v, Vector2d onto) {
        double normSquared = onto.sqrNorm();
        if (normSquared == 0.0) {
            throw new IllegalArgumentException("Projection requires a nonzero vector");
        }
        return onto.times(v.dot(onto) / normSquared);
    }
    // end sample

    // sample: vectorRotation
    public static Vector2d rotated(Vector2d v, double angle) {
        return Rotation2d.exp(angle).times(v);
    }
    // end sample

    // sample: vectorBearing
    public static double bearingTo(Vector2d from, Vector2d to) {
        Vector2d delta = to.minus(from);
        if (delta.x == 0.0 && delta.y == 0.0) {
            throw new IllegalArgumentException("Bearing requires distinct positions");
        }
        return Math.atan2(delta.y, delta.x);
    }
    // end sample
}

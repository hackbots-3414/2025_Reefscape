package frc.robot.utils;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meters;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotObserver;

public class Shape {
    private final List<Translation2d> m_vertices;

    public Shape(List<Translation2d> vertices) {
        this.m_vertices = vertices;
    }

    public static Shape fromUnsortedVertices(List<Translation2d> unsortedVertices) {
        if (unsortedVertices.size() < 3) {
            throw new IllegalArgumentException("Polygon must be atleat 3 points.");
        }

        List<Translation2d> mutableList = new ArrayList<>(unsortedVertices);

        double centerX = 0, centerY = 0;
        for (Translation2d vertex : unsortedVertices) {
            centerX += vertex.getX();
            centerY += vertex.getY();
        }

        centerX /= unsortedVertices.size();
        centerY /= unsortedVertices.size();

        final double cx = centerX;
        final double cy = centerY;

        mutableList.sort(Comparator.comparingDouble(point -> Math.atan2(point.getY() - cy, point.getX() - cx)));

        return new Shape(mutableList);
    }

    // ray cast: if crosses odd times, its inside
    public boolean isPointInside (Translation2d point) {
        int crossings = 0;
        int numVertices = m_vertices.size();

        // horizontal ray pointing right, counts num vertices it crosses
        for (int i = 0; i < numVertices; i++) {
            Translation2d start = m_vertices.get(i);
            Translation2d end = m_vertices.get((i+1) % numVertices); // rolls over to 1 after last vertex

            // converted to doubles for ez math
            double x = point.getX(), y = point.getY();
            double x1 = start.getX(), y1 = start.getY();
            double x2 = end.getX(), y2 = end.getY();

            if ((y > y1) != (y > y2)) { // if both true? not within y1-y2 bounds
                // calculates the x coordinate of where it intersects the line
                double intersectionPoint = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
                if (x < intersectionPoint) { // we only care if its to the RIGHT (when point less than intersection point)
                    crossings++;
                }
            }
        }

        return (crossings % 2 == 1);
    }

    public boolean isActive() {
        if (RobotObserver.getVisionExpired()) {
            return true;
        }
        return isPointInside(FieldUtils.flipPose(RobotObserver.getPose()).getTranslation());
    }

    public List<Translation2d> getVertices() {
        return m_vertices;
    }

    // flips hotdog style (up down) - ex. reef pose
    public static Shape flipHotdog(Shape shape) {
        // flips the y axis by doing field width - old y
        return new Shape(shape.getVertices().stream()
            .map(oldTranslation -> new Translation2d(oldTranslation.getX(), FieldConstants.k_fieldWidth.in(Meters) - oldTranslation.getY()))
            .collect(Collectors.toList()));
    }
}
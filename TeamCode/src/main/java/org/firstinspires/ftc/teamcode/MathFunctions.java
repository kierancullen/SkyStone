package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class MathFunctions {
    public static double AngleWrap (double angle) {
        while (angle < -Math.PI) {
            angle += 2*Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2*Math.PI;
        }

        return angle;
    }

    public static ArrayList lineCircleIntersection (Point2D circleCenter, double circleRadius, Point2D lineStart, Point2D lineEnd) {
        double slope = ((lineEnd.y-lineStart.y)/(lineEnd.x-lineStart.x));

        //Offset the line endpoints so that we can consider the center of the circle to be at (0,0):
        Point2D lineStartOffset = new Point2D((lineStart.x - circleCenter.x), (lineStart.y - circleCenter.y));
        Point2D lineEndOffset = new Point2D((lineEnd.x - circleCenter.x), (lineEnd.y - circleCenter.y));

        double yIntercept = (lineStart.y-(slope * lineStart.x));
        double yInterceptOffset = (lineStartOffset.y-(slope * lineStartOffset.x));

        //From the solved quadratic for the line-circle intersection:
        double term1 = (yInterceptOffset * slope);
        double term2 = Math.sqrt(Math.pow(circleRadius, 2) + (Math.pow(circleRadius, 2) * Math.pow(slope, 2)) - Math.pow(yInterceptOffset, 2));
        double term3 = Math.pow(slope, 2) + 1;

        ArrayList<Point2D> intersections = new ArrayList<>();

        try {
            //Find the roots and use them to calculate the intersection points:
            double root1 = (-term1 + term2) / term3;
            root1 += circleCenter.x; //Remove offset
            Point2D intersection1 = new Point2D(root1, (slope*root1 + yIntercept));
            //Check if the intersection is on the line segment:
            double minX = Math.min(lineEnd.x, lineStart.x);
            double maxX = Math.max(lineEnd.x, lineStart.x);
            if (intersection1.x > minX && intersection1.x < maxX) {
                intersections.add(intersection1);
            }

            //Repeat for the second point:
            double root2 = -((term1 + term2) / term3);
            root2 += circleCenter.x;
            Point2D intersection2 = new Point2D(root2, (slope*root2 + yIntercept));
            if (intersection2.x > minX && intersection2.x < maxX) {
                intersections.add(intersection2);
            }
        }
        catch (Exception e) {
            //There were no intersections :(
        }

        return intersections;
    }

    public static Node extendLine (Node startLine, Node endLine, double extendDist) {
        double length = Math.sqrt(Math.pow(startLine.locationY - endLine.locationY, 2) + Math.pow(startLine.locationX - endLine.locationX, 2));
        double extendedLength = length + extendDist;
        double extendX = (endLine.locationX - startLine.locationX) / length * extendedLength;
        double extendY = (endLine.locationY - startLine.locationY) / length * extendedLength;
        Node extended = new Node(endLine);
        extended.setPoint(new Point2D(endLine.locationX + extendX, endLine.locationY + extendY));
        return extended;
    }

    public static boolean onSegment (Node point, Node startLine, Node endLine) {
        double slope = ((endLine.locationY - startLine.locationY) / (endLine.locationX - startLine.locationX));
        double minX = Math.min(endLine.locationX, startLine.locationX);
        double maxX = Math.max(endLine.locationX, startLine.locationX);
        boolean inRange = (point.locationX >= minX && point.locationX <= maxX);
        return (Math.abs((point.locationY - endLine.locationY - slope*(point.locationX - endLine.locationX))) < 0.1 && inRange);
    }


}

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.Range;
import static org.firstinspires.ftc.teamcode.GlobalPosition.*;
import static org.firstinspires.ftc.teamcode.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.MathFunctions.extendLine;
import static org.firstinspires.ftc.teamcode.MathFunctions.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.GlobalMovement.*;

import java.util.ArrayList;


public class RobotMovement {


    public static void runPurePursuit(Path input, double followAngle, double stopTolerance) {

        input.updateProgress(); //Update the Patg's data about what nodes we've passed


        if (input.isExtended == false) {
            Node newFinalPoint = extendLine(input.nodes.get(input.nodes.size()-2), input.nodes.get(input.nodes.size()-1), input.nodes.get(input.nodes.size()-2).followDistance); //Get the location of our new final point by extending the segment between the last few points by the follow distance of the second-to-last point
            input.nodes.add(newFinalPoint);

        }

        Node current = input.nodes.get(input.passedNode);
        Node target = input.nodes.get(input.passedNode + 1);  //The point we want to drive to on this iteration. By default it's the next node on the path, so if we're off the path and don't find any other intersections, we can just go there
        Node stop = input.nodes.get(input.nodes.size()-2); //The point we want to stop at (it's the second-to-last because we extended the path)
        Double distanceToStop = Math.sqrt(Math.pow(stop.locationX - worldPosition_x, 2) + Math.pow(stop.locationY - worldPosition_y, 2));




        for (int i = input.passedNode; i <= input.passedNode + 1; i++) { //We're only searching for valid intertsections on the segment that we're currently on, and on the next segment that we haven't reached yet
            Node startLine = input.nodes.get(i);
            Node endLine = input.nodes.get(i+1);
            Point2D robotLocation = new Point2D(worldPosition_x, worldPosition_y);
            ArrayList<Point2D> intersections = lineCircleIntersection(robotLocation, startLine.followDistance, startLine.toPoint(), endLine.toPoint());

            double closestDistance = 1000000000;

            for(Point2D thisIntersection: intersections) {
                double distanceToNext =  Math.sqrt(Math.pow(thisIntersection.x - input.nodes.get(input.passedNode + 1).locationX, 2) + Math.pow(thisIntersection.x - input.nodes.get(input.passedNode + 1).locationX, 2)); //Distance between the current intersection we're checking and the NEXT node that we haven't gone through yet
                if (distanceToNext < closestDistance) {
                    closestDistance = distanceToNext;
                    target = new Node(startLine); //Create a new Node that's a copy of the point at the beginning of the segment so that we have all the speed settings
                    target.setPoint(thisIntersection); //Change the location of the node to the best intersection
                }
            }

        }


        //If we're going to a point on the last segment and we're close enough to the stop point, set the target point so that we go directly there.
        boolean slowDown = false; //Controls whether we want a power decay as we drive to the target. This is only true if we're moving to the stop point/

        if (input.passedNode >= input.nodes.size() - 3 && distanceToStop <= input.nodes.get(input.passedNode).followDistance) {
            target = new Node(stop); //We're using the speed settings from the stop point for this move, but we'll make a copy
            target.setPoint(new Point2D(stop.locationX, stop.locationY));
            slowDown = true;
        }

        //System.out.println("(" + target.locationX + ", " + target.locationY + ")");
        goToPosition(target.locationX, target.locationY, target.moveSpeed, followAngle, target.turnSpeed, slowDown);

    }

    public static double goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, boolean slowDown) {

        double distanceToTarget = Math.hypot(x-worldPosition_x, y-worldPosition_y);

        //System.out.println(movement_x);
        //System.out.println(movement_y);

        double absoluteAngleToTarget = Math.atan2(y-worldPosition_y, x-worldPosition_x);
        //System.out.println("absoluteAngleToTarget: " + Math.toDegrees(absoluteAngleToTarget));


        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget -worldAngle_rad);
        //System.out.println("relativeAngleToPoint: " + Math.toDegrees(relativeAngleToPoint));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        //System.out.println("relativeXToPoint: " + relativeXToPoint);
        //System.out.println("relativeYToPoint: " + relativeYToPoint);

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        if (slowDown) {
            movement_x = movementXPower * movementSpeed
                    * (movementSpeed * Range.clip(distanceToTarget / 10, 0, 1)); //Slow down when we get close to the target to avoid overshooting
            movement_y = movementYPower * movementSpeed
                    * (movementSpeed * Range.clip(distanceToTarget / 10, 0, 1));
        }

        else {
            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;
        }

        if (distanceToTarget < 0.1) {
            movement_x = 0;
            movement_y = 0;
        }

        double relativeTurnAngle = relativeAngleToPoint + preferredAngle;
        System.out.println(relativeTurnAngle);

        //Stop turning when we're close to the target to avoid "orbiting" if we overshoot

        if (distanceToTarget < 10) movement_turn = 0;
        else movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;


        return distanceToTarget;

    }

}
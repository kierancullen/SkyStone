package org.firstinspires.ftc.teamcode;


public class Node {
    public double locationX;
    public double locationY;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;

    //When going to this node, we can point the robot to a different location (helpful in some circumstances)
    public double headingX;
    public double headingY;



    public Node(double locationX, double locationY, double moveSpeed, double turnSpeed, double followDistance) {

        this.locationX = locationX;
        this.locationY = locationY;
        this.headingX = locationX;
        this.headingY = locationY;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;

    }

    //Same constructor as before, except it sets the heading point to something other than the location

    public Node(double locationX, double locationY, double headingX, double headingY, double moveSpeed, double turnSpeed, double followDistance) {

        this.locationX = locationX;
        this.locationY = locationY;
        this.headingX = headingX;
        this.headingY = headingY;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;

    }

    public Node(Node input) {

        locationX = input.locationX;
        locationY = input.locationY;
        headingX = input.headingX;
        headingY = input.headingY;
        moveSpeed = input.moveSpeed;
        turnSpeed = input.turnSpeed;
        followDistance = input.followDistance;
    }

    public Point2D toPoint () {

        return new Point2D(locationX, locationY);
    }

    public void setPoint(Point2D point) {
        locationX = point.x;
        locationY = point.y;

    }
}


package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import static org.firstinspires.ftc.teamcode.GlobalPosition.*;


public class Path {

    public ArrayList<Node> nodes;
    public int passedNode;
    public boolean isExtended;

    public Path () {
        nodes = new ArrayList<Node>();
        passedNode = 0;
        isExtended = false;
    }

    public Path (ArrayList<Node> inputNodes) {
        nodes = inputNodes;
        passedNode=0;
        isExtended = false;
    }

    public void updateProgress () {
        Point2D location = new Point2D(worldPosition_x, worldPosition_y);

    }


}

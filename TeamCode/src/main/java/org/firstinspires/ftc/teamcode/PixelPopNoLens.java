
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;


public class PixelPopNoLens {

    public static int[][] STONE_LOCATIONS_BLUE = new int[][]{
            {608, 430},
            {362, 431},
            {87, 432},
    };


    public static int[][] STONE_LOCATIONS_RED = new int[][]{
            {80, 426},
            {340, 426},
            {586, 423},
    };

    private static final String VUFORIA_KEY =
            "Ac0A5xL/////AAABmbaZRuKrykmMhgpBAfm4wxkWMeMkHp/ij0Bv8cnqyigZaQN4qUU9wK+CmT4WDTRnZef/AEyluCOS1Z8a5pwiHeJpjLNqVcQoQsXBJT06NyKXZ2v2BDMqURXAnLCl82w+vIY3u4W/XdtFBt2m0/5OQNLFZRaIz3LJZaXGYz4hSRFAyMj0yVonukAXvjQljMxjd1YNUhpXk8V3qJaXS49Ep69t0AypLu+hE2AdHg1e15q29AifPAANhWM0PpWEACCVn7RWe19wyNi6N8Ab0c77kudZoGWmQF4hZVGRKK3ZrVz7kz1wyk3tfzHUsteJm7hbw8kagADt2ZKBDkO4+0i0HtB2hXcrKUp/w23nNTtY4SJ0";

    private VuforiaLocalizer vuforia;

    /*public void runOpMode() {

        initVuforia();

        waitForStart();

        /*sleep(5000);

            vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
                @Override
                public void accept(Frame frame) {
                    Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                    if (bitmap != null) {
                        File file = new File(AppUtil.FIRST_FOLDER, "boi.png");
                        try {
                            FileOutputStream outputStream = new FileOutputStream(file);
                            try {
                                bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                            } finally {
                                outputStream.close();
                                telemetry.addData("doen", "");
                                telemetry.update();
                            }
                        } catch (IOException e) {
                            telemetry.addData("fale","");
                            telemetry.update();
                        }
                    }
                }
            }));

            while(opModeIsActive()) {sleep(1);telemetry.update();}

        while (opModeIsActive()) {

            vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
                @Override
                public void accept(Frame frame) {
                    Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                    if (bitmap != null) {
                        int[] locations = getLocations(bitmap);
                        telemetry.addData("First stone", locations[0]);
                        telemetry.addData("Second stone", locations[1]);
                        telemetry.update();
                    }
                }
            }));
        }

    }*/

    public int[] locations;

    public void storeLocations(int[] locations) {

        //Pattern A
        if (locations[0] == 2) {
            this.locations = new int[]{0, 3};
        }

        //Pattern B
        else if (locations[0] == 0) {
            this.locations = new int[]{2, 5};
        }

        //Pattern C
        else if (locations[0] == 1) {
            this.locations = new int[]{1, 4};
        }
    }

    public void captureLocations(final int[][] coordsArr){
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    storeLocations(getLocations(bitmap, coordsArr));
                }
            }
        }));
    }



    public int[] getLocations(Bitmap frame, int[][] coordsArr){
        int bestFirstLocation = -1;
        double bestDifference = 0;
        for (int firstLocation = 0; firstLocation < 3; firstLocation++) {
            int firstPixel = frame.getPixel(coordsArr[firstLocation][0], coordsArr[firstLocation][1]);
            int firstR = Color.red(firstPixel);
            int firstG = Color.green(firstPixel);
            int firstB = Color.blue(firstPixel);
            int selAvgR = (int)(firstR);
            int selAvgG = (int)(firstG);
            int selAvgB = (int)(firstB);
            int othAvgR = 0;
            int othAvgG = 0;
            int othAvgB = 0;
            int nOth = 0;
            for (int i = 0; i < 3; i++) {
                if (i != firstLocation) {
                    nOth++;
                    int thisPixel = frame.getPixel(coordsArr[i][0], coordsArr[i][1]);
                    othAvgR += Color.red(thisPixel);
                    othAvgG += Color.green(thisPixel);
                    othAvgB += Color.blue(thisPixel);

                }
            }
            othAvgR /= nOth;
            othAvgG /= nOth;
            othAvgB /= nOth;
            double distance = 0;
            distance += Math.abs(selAvgR - othAvgR);
            distance += Math.abs(selAvgG - othAvgG);
            distance += Math.abs(selAvgB - othAvgB);
            if (distance > bestDifference) {
                bestDifference = distance;
                bestFirstLocation = firstLocation;
            }
        }
        return new int[] {bestFirstLocation};
    }


    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();

    }

}
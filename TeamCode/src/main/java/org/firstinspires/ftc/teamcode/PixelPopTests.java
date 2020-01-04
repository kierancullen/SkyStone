
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


@TeleOp(name = "PixelPopTests")
//@Disabled
public class PixelPopTests extends LinearOpMode {

    public static int[][] STONE_LOCATIONS_BLUE = new int[][]{
            {120, 381},
            {168, 381},
            {245, 382},
            {354, 381},
            {489, 385},
            {622, 387},


    };


    public static int[][] STONE_LOCATIONS_RED = new int[][]{
            {907, 375},
            {821, 378},
            {728, 380},
            {611, 381},
            {479, 383},
            {350, 386},
    };

    private static final String VUFORIA_KEY =
            "Ac0A5xL/////AAABmbaZRuKrykmMhgpBAfm4wxkWMeMkHp/ij0Bv8cnqyigZaQN4qUU9wK+CmT4WDTRnZef/AEyluCOS1Z8a5pwiHeJpjLNqVcQoQsXBJT06NyKXZ2v2BDMqURXAnLCl82w+vIY3u4W/XdtFBt2m0/5OQNLFZRaIz3LJZaXGYz4hSRFAyMj0yVonukAXvjQljMxjd1YNUhpXk8V3qJaXS49Ep69t0AypLu+hE2AdHg1e15q29AifPAANhWM0PpWEACCVn7RWe19wyNi6N8Ab0c77kudZoGWmQF4hZVGRKK3ZrVz7kz1wyk3tfzHUsteJm7hbw8kagADt2ZKBDkO4+0i0HtB2hXcrKUp/w23nNTtY4SJ0";

    private VuforiaLocalizer vuforia;

    public void runOpMode() {

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

            while(opModeIsActive()) {sleep(1);telemetry.update();} */

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

    }

    public int[] getLocations(Bitmap frame){
        int bestFirstLocation = -1;
        int bestSecondLocation = -1;
        double bestDifference = 0;
        for (int firstLocation = 0; firstLocation < 6; firstLocation++) {
            int firstPixel = frame.getPixel(STONE_LOCATIONS_RED[firstLocation][0], STONE_LOCATIONS_RED[firstLocation][1]);
            int firstR = Color.red(firstPixel);
            int firstG = Color.green(firstPixel);
            int firstB = Color.blue(firstPixel);
            for (int secondLocation = 0; secondLocation < 6; secondLocation++) {
                if (secondLocation == firstLocation) { continue; }
                int secondPixel = frame.getPixel(STONE_LOCATIONS_RED[secondLocation][0], STONE_LOCATIONS_RED[secondLocation][1]);
                int secondR = Color.red(secondPixel);
                int secondG = Color.green(secondPixel);
                int secondB = Color.blue(secondPixel);
                int selAvgR = (int)(firstR + secondR) / 2;
                int selAvgG = (int)(firstG + secondG) / 2;
                int selAvgB = (int)(firstB + secondB) / 2;
                int othAvgR = 0;
                int othAvgG = 0;
                int othAvgB = 0;
                int nOth = 0;
                for (int i = 0; i < 6; i++) {
                    if (i != firstLocation && i != secondLocation) {
                        nOth++;
                        int thisPixel = frame.getPixel(STONE_LOCATIONS_RED[i][0], STONE_LOCATIONS_RED[i][1]);
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
                    bestSecondLocation = secondLocation;
                }
            }
        }
        return new int[] {bestFirstLocation, bestSecondLocation};
    }


    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();

    }

}
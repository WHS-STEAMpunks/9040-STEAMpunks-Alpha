package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.classifier.Classifications;

import java.io.File;
import java.util.List;


@Autonomous(name = "Radium")
public class Radium extends LinearOpMode
{
    final int RESOLUTION_WIDTH = 1280;
    final int RESOLUTION_HEIGHT = 720;
    private ElapsedTime timer;
    private VisionPortal portal;
    private CameraName camera;
    private ImageRecognition classifier;
    private int label, frameNum;
    private long inferenceTime;

    @Override
    public void runOpMode()
    {
        File dir = new File("/sdcard/VisionPortal-Capture");
        String[] children = dir.list();
        for (int i = 0; i < children.length; i++)
        {
            boolean deleted = new File(dir, children[i]).delete();
            telemetry.addData("File Delete", "" + deleted);
            telemetry.update();
        }
        timer = new ElapsedTime();
        camera = hardwareMap.get(WebcamName.class, "cool cam");
        portal = new VisionPortal.Builder().setCamera(camera).setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT)).build();
        portal.stopLiveView();
        classifier = new ImageRecognition(0.5f, 1, 3, 0, 0);
        frameNum = 0;
        label = 0;
        List<Classifications> results;
        List<Category> categories;
        telemetry.addData("Camera", "Ready");
        telemetry.update();
        timer.reset();

        while (opModeInInit())
        {
            ((VisionPortalImpl)portal).saveNextFrameRaw("Capture/" + frameNum);
            sleep(500);
            File input = new File("/sdcard/VisionPortal-Capture/" + frameNum + ".png");
            sleep(500);
            Bitmap bitmap = classifier.PNG2BMP(input);
            sleep(500);
            results = classifier.classify(bitmap, 0);
            telemetry.addData("Results", results);
            telemetry.update();
            while (results == null && opModeIsActive())
            {
                telemetry.addData("Recognition", "Null");
                telemetry.update();
                frameNum++;
                ((VisionPortalImpl)portal).saveNextFrameRaw("Capture/" + frameNum);
                sleep(500);
                input = new File("/sdcard/VisionPortal-Capture/" + frameNum + ".png");
                sleep(500);
                bitmap = classifier.PNG2BMP(input);
                sleep(500);
                results = classifier.classify(bitmap, 0);
            }
            for (Classifications detection : results)
            {
                categories = detection.getCategories();
                Category correctDetection = null;
                for (Category x : categories)
                {
                    if (correctDetection == null) {correctDetection = x;}
                    else if (x.getScore() > correctDetection.getScore()) {correctDetection = x;}
                }
                if (correctDetection == null) {break;}
                inferenceTime = classifier.getInferenceTime();
                telemetry.addData("Recognition", correctDetection.getDisplayName());
                telemetry.addLine(correctDetection.getLabel() + ": " + correctDetection.getScore());
                telemetry.addLine("Inference Time: " + inferenceTime);
                label = Integer.parseInt(correctDetection.getLabel());
                telemetry.update();
            }
            frameNum++;
        }
        classifier.clearImageClassifier();
        if (label == 0)
        {
            telemetry.addData("Recognition", "Left");
            telemetry.update();
        }
        else if (label == 1)
        {
            telemetry.addData("Recognition", "Middle");
            telemetry.update();
        }
        else
        {
            telemetry.addData("Recognition", "Right");
            telemetry.update();
        }
    }
    public VisionPortalImpl getPortal() {return (VisionPortalImpl) portal;}
    public CameraName getCamera() {return camera;}
}
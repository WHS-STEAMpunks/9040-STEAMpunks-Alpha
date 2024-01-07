package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.classifier.Classifications;

import java.io.File;
import java.util.List;

@Autonomous (name = "Red Front")
public class redFront extends LinearOpMode
{
    // Constants
    private static final double BACKDROP_SAFETY_DISTANCE = 12.0; // in inches
    private static final Float INFERENCE_CONFIDENCE_THRESHOLD = 0.5f;
    private final int RESOLUTION_WIDTH = 1280;
    private final int RESOLUTION_HEIGHT = 720;
    private final static double BOX_TURNER_HOME = 0.5;
    private final static double BOX_TURNER_RANGE = 0.28;
    private final static double BOX_OPENER_HOME = 0.5;
    private final static double BOX_OPENER_RANGE = 0.108;
    private final static double HIGH_SET_LINE = 25.75; // in inches
    private final static double MEDIUM_SET_LINE = 19; // in inches
    private final static double LOW_SET_LINE = 12.375; // in inches
    private final static double WHEEL_DIAMETER = 4.0; // in inches
    private final static double SLIDE_HUB_DIAMETER = 1.5; // in inches
    private final static double TICKS_PER_REV = 537.7;
    private final static String MODEL_NAME = "redBack.tflite";


    // Computer Vision
    private VisionPortal portal;
    private CameraName camera;
    private ImageRecognition classifier;
    private AprilTagProcessor aprilTagProcessor;
    private boolean targetFound;
    private AprilTagDetection desiredTag;
    private List<Classifications> results;
    private List<Category> categories;
    private List<AprilTagDetection> currentDetections;
    private int desiredTagID;
    private int label, frameNum;
    private long inferenceTime;


    // Motors
    private DcMotorEx frontleft;
    private DcMotorEx backright;
    private DcMotorEx backleft;
    private DcMotorEx frontright;
    private DcMotorEx intake;
    private DcMotorEx slides;
    private Servo boxTurner;
    private Servo boxOpener;


    // Sensors
    //private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currentAngle;
    private TouchSensor touchSensor;


    // Positions
    private double xPos; // in inches
    private double yPos; // in inches
    private int slidePos;


    // Essentially the main method
    public void runOpMode()
    {
        // Initialization
        initCV();
        initMotors();
        initSensors();
        telemetry.addData("C4", "Ready");
        telemetry.update();

        // Recognize position of prop on spike marks during Init, takes the latest recognition
        while (opModeInInit()) {recognizePosition();}
        desiredTagID = label;

        // Clear up memory
        classifier.clearImageClassifier();

        forward(48);
        right(96);
    }

    // Computer Vision Methods
    public void initCV()
    {
        // Clear capture directory for new webcam pictures
        File dir = new File("/sdcard/VisionPortal-Capture");
        String[] children = dir.list();
        for (int i = 0; i < children.length; i++)
        {
            boolean deleted = new File(dir, children[i]).delete();
            telemetry.addData("File Delete", "" + deleted);
            telemetry.update();
        }

        // Initialize Computer Vision Stuff
        camera = hardwareMap.get(WebcamName.class, "cool cam");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor.setDecimation(2);
        portal = (new VisionPortal.Builder().setCamera(camera).setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT)).addProcessor(aprilTagProcessor)).build();
        portal.stopLiveView();
        classifier = new ImageRecognition(INFERENCE_CONFIDENCE_THRESHOLD, 1, 3, 0, 0, MODEL_NAME);
        frameNum = 0;
        label = -1;
        desiredTagID = -1;
        targetFound = false;
        desiredTag  = null;
        currentDetections = null;
    }

    public void initMotors()
    {
        // Initialize Motors
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backright = hardwareMap.get(DcMotorEx.class, "back right");
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        // May need to change this depending on how robot behaves
        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        backleft.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);

        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slides.setDirection(DcMotorEx.Direction.REVERSE);

        boxTurner = hardwareMap.get(Servo.class, "box turner");
        boxTurner.setDirection(Servo.Direction.FORWARD);
        boxTurner.setPosition(BOX_TURNER_HOME);

        boxOpener = hardwareMap.get(Servo.class, "box opener");
        boxOpener.setDirection(Servo.Direction.FORWARD);
        boxOpener.setPosition(BOX_OPENER_HOME);
    }

    public void initSensors()
    {
        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        currentAngle = 0.0;*/
        touchSensor = hardwareMap.get(TouchSensor.class, "touch sensor");
    }

    public void recognizePosition()
    {
        ((VisionPortalImpl)portal).saveNextFrameRaw("Capture/" + frameNum);
        sleep(250);
        File input = new File("/sdcard/VisionPortal-Capture/" + frameNum + ".png");
        sleep(250);
        Bitmap bitmap = classifier.PNG2BMP(input);
        sleep(250);
        results = classifier.classify(bitmap, 0);
        telemetry.addData("Results", results);
        telemetry.update();
        while (results == null)
        {
            telemetry.addData("Recognition", "Null");
            telemetry.update();
            frameNum++;
            ((VisionPortalImpl)portal).saveNextFrameRaw("Capture/" + frameNum);
            sleep(250);
            input = new File("/sdcard/VisionPortal-Capture/" + frameNum + ".png");
            sleep(250);
            bitmap = classifier.PNG2BMP(input);
            sleep(250);
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

    public void goToCorrectAprilTag()
    {
        currentDetections = aprilTagProcessor.getDetections();
        sleep(250);
        while (currentDetections == null || currentDetections.size() == 0)
        {
            currentDetections = aprilTagProcessor.getFreshDetections();
            sleep(250);
        }
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.update();

        for (AprilTagDetection detection : currentDetections)
        {
            if (targetFound) {break;}
            else if (detection.metadata != null && detection.id == desiredTagID)
            {
                targetFound = true;
                telemetry.addLine("ID: " + detection.id + ", " + detection.metadata.name);
                desiredTag = detection;
            }
            else
            {
                telemetry.addData("Unknown Label", detection.id);
                telemetry.update();
            }
        }
        telemetry.update();
        if (targetFound)
        {
            /*// Sets current mode to using encoders
            frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            double  rangeError = desiredTag.ftcPose.y;
            double  bearingError = desiredTag.ftcPose.bearing;
            while (rangeError > BACKDROP_SAFETY_DISTANCE)
            {
                if (bearingError > 15)
                {
                    // strafe left
                    power(-0.3, 0.3, 0.3, -0.3);
                }
                else if (bearingError < -15)
                {
                    // strafe right
                    power(0.3, -0.3, -0.3, 0.3);
                }
                else
                {
                    // forward
                    power(acceleratorTransform(rangeError));
                }
                targetFound = false;
                currentDetections = aprilTagProcessor.getDetections();
                sleep(250);
                while (currentDetections == null || currentDetections.size() == 0)
                {
                    currentDetections = aprilTagProcessor.getFreshDetections();
                    sleep(250);
                }
                for (AprilTagDetection detection : currentDetections)
                {
                    if (targetFound) {break;}
                    else if (detection.metadata != null && detection.id == desiredTagID)
                    {
                        targetFound = true;
                        desiredTag = detection;
                    }
                }
                rangeError = desiredTag.ftcPose.range;
                bearingError = desiredTag.ftcPose.bearing;
            }
            power(0);
            sleep(50);*/
            right(desiredTag.ftcPose.x);
            slidesUp(1);
            boxOut();
            forward(desiredTag.ftcPose.y - BACKDROP_SAFETY_DISTANCE);
        }
    }


    // Motor Methods
    public void power(double pow)
    {
        frontleft.setPower(pow);
        frontright.setPower(pow);
        backleft.setPower(pow);
        backright.setPower(pow);
    }

    public void power(double fL, double fR, double bL, double bR)
    {
        frontleft.setPower(fL);
        frontright.setPower(fR);
        backleft.setPower(bL);
        backright.setPower(bR);
    }

    public double driveProgress()
    {
        double fL = Math.abs(((double)frontleft.getCurrentPosition() / frontleft.getTargetPosition()));
        //double fR = Math.abs(((double)frontright.getCurrentPosition() / frontright.getTargetPosition()));
        double bL = Math.abs((double)backleft.getCurrentPosition() / backleft.getTargetPosition());
        //double bR = Math.abs(((double)backright.getCurrentPosition() / backright.getTargetPosition()));
        return ((fL + bL) / 2);
    }

    public void forward(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Sets current mode to using encoders
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backleft.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //frontright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //backright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Forward", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5 && opModeIsActive())
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
        error /= 2.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while ((frontleft.isBusy() || backleft.isBusy()) && opModeIsActive())
        {
            telemetry.addData("Forward", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
            error /= 2.0;
            error = (error / TICKS_PER_REV) * Math.PI;
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


        // Cut off power
        power(0.0);
        sleep(50);
        yPos += distance;
    }

    public void backward(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Sets current mode to using encoders
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backleft.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //frontright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //backright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Backward", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5 && opModeIsActive())
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
        error /= 2.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while ((frontleft.isBusy() || backleft.isBusy()) && opModeIsActive())
        {
            telemetry.addData("Backward", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
            error /= 2.0;
            error = (error / TICKS_PER_REV) * Math.PI;
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


        // Cut off power
        power(0.0);
        sleep(50);
        yPos -= distance;
    }

    public void left(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Sets current mode to using encoders
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backleft.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //frontright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //backright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection((DcMotorEx.Direction.REVERSE));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Left", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5 && opModeIsActive())
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
        error /= 2.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while ((frontleft.isBusy() || backleft.isBusy()) && opModeIsActive())
        {
            telemetry.addData("Left", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
            error /= 2.0;
            error = (error / TICKS_PER_REV) * Math.PI;
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


        // Cut off power
        power(0.0);
        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        backleft.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
        sleep(50);
        xPos -= distance;
    }

    public void right(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Sets current mode to using encoders
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backleft.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //frontright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //backright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection((DcMotorEx.Direction.FORWARD));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Right", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5 && opModeIsActive())
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
        error /= 2.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while ((frontleft.isBusy() || backleft.isBusy()) && opModeIsActive())
        {
            telemetry.addData("Right", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
            error /= 2.0;
            error = (error / TICKS_PER_REV) * Math.PI;
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


        // Cut off power
        power(0.0);
        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        backleft.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
        sleep(50);
        xPos += distance;
    }

    public void resetAngle()
    {
        //lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        //currentAngle = 0.0;
    }

    public double getAngle()
    {
        //Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        //double deltaTheta = orientation.firstAngle - lastAngles.firstAngle;
        //if (deltaTheta > 180) {deltaTheta -= 360;}
        //else if (deltaTheta <= -180) {deltaTheta += 360;}
        //currentAngle += deltaTheta;
        //lastAngles = orientation;
        return currentAngle;
    }

    public void clockwise(double degrees)
    {
        /*resetAngle();
        double pow = 0.6;
        double error = degrees;
        while (Math.abs(error) > 2 && opModeIsActive())
        {
            pow = acceleratorTransform(error);
            if (error < 0) {power(pow, -pow, pow, -pow);}
            else {power(-pow, pow, -pow, pow);}
            error = degrees - getAngle();
        }
        power(0);*/
    }

    public void counterclockwise(double degrees)
    {
        /*resetAngle();
        double pow = 0.6;
        double error = degrees;
        while (Math.abs(error) > 2 && opModeIsActive())
        {
            pow = acceleratorTransform(error);
            if (error < 0) {power(-pow, pow, -pow, pow);}
            else {power(pow, -pow, pow, -pow);}
            error = degrees - getAngle();
        }
        power(0);*/
    }

    public void slidesUp(int level)
    {
        // Reset encoder counts to 0
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        if (level == 3)
        {
            slidePos = (int)((HIGH_SET_LINE / (SLIDE_HUB_DIAMETER * Math.PI)) * TICKS_PER_REV);
            slides.setTargetPosition(slidePos);
        }
        else if (level == 2)
        {
            slidePos = (int)((MEDIUM_SET_LINE / (SLIDE_HUB_DIAMETER * Math.PI)) * TICKS_PER_REV);
            slides.setTargetPosition(slidePos);
        }
        else
        {
            slidePos = (int)((LOW_SET_LINE / (SLIDE_HUB_DIAMETER * Math.PI)) * TICKS_PER_REV);
            slides.setTargetPosition(slidePos);
        }

        // Set the motors to move to the specified positions
        slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slides.setPower(0.8); // set power for moving up
        while (slides.isBusy() && opModeIsActive())
        {
            telemetry.addData("Slides Up", level);
            telemetry.update();
        }

        // Cut off power momentarily and switch modes
        slides.setPower(0);
        power(0.0);
        slides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void slideStop() {slides.setPower(0);}

    public void slidesDown()
    {
        // reset encoder counts to 0
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        while (!(touchSensor.isPressed()) && opModeIsActive()) {slides.setPower(-0.3);}

        // Cut off power
        slides.setPower(0.0);
        slides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slidePos = 0; // update position of linear slides
    }

    public void goToX(double x) // goes to the vertical line (x coordinate) specified
    {
        double dx = x - xPos;
        right(dx);
    }
    public void goToY(double y) // goes to the horizontal line (y coordinate) specified
    {
        double dy = y - yPos;
        forward(dy);
    }

    public double acceleratorTransform(double input)
    {
        return Math.tanh(0.36 + 0.05 * input);
        // Play around with these numbers until they work
        // 0.6 is where the maximum possible power for the robot should go
        // 0.25 is where the minimum possible power for the robot should go
        // 0.05 is how much the power should increase/decrease for every inch traveled
        // input should be the number of inches traveled or the number of inches away from the target
    }

    public void boxOut() {boxTurner.setPosition(BOX_TURNER_RANGE);}
    public void boxIn() {boxTurner.setPosition(BOX_TURNER_HOME);}
    public void openBox() {boxOpener.setPosition(BOX_OPENER_RANGE);}
    public void closeBox() {boxOpener.setPosition(BOX_OPENER_HOME);}

}
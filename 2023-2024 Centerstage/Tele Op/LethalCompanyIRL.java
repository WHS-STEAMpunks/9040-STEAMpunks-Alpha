package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.classifier.Classifications;

import java.io.File;
import java.util.List;


@TeleOp(name = "Lethal Company irl")
public class LethalCompanyIRL extends LinearOpMode
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
    private final static double HANGER_HOME = 0.6;
    private final static double HANGER_RANGE = 0.01;
    private final static double LAUNCHER_HOME = 0.5;
    private final static double LAUNCHER_RANGE = 0.11;
    private final static double HIGH_SET_LINE = 20.1; // in inches
    private final static double MEDIUM_SET_LINE = 16; // in inches
    private final static double LOW_SET_LINE = 12.375; // in inches
    private final static double WHEEL_DIAMETER = 4.0; // in inches
    private final static double SLIDE_HUB_DIAMETER = 1.5; // in inches
    private final static double TICKS_PER_REV = 537.7;
    private final static double pow = 0.77;
    private final static double slowPow = 0.2;
    private final static double intakePow = 0.72;


    // Motors
    private DcMotorEx frontleft;
    private DcMotorEx backright;
    private DcMotorEx backleft;
    private DcMotorEx frontright;
    private DcMotorEx intake;
    private DcMotorEx actuator;
    private DcMotorEx slides;
    private Servo hanger;
    private Servo launcher;
    private Servo boxTurner;
    private Servo boxOpener;


    // Sensors
    private IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currentAngle;
    private TouchSensor touchSensor;


    // Positions
    private double xPos; // in inches
    private double yPos; // in inches
    private int slidePos;


    // Controls
    private double stickAngle;
    private boolean forward, backward, left, right, slowForward, slowBackward, slowLeft, slowRight;
    private boolean diagonal45, diagonal135, diagonal225, diagonal315, quickTurnLeft, quickTurnRight;
    private boolean turnLeft, turnRight, stopDrive, actuatorUp, actuatorDown, actuatorOff;
    private boolean launch, hangUp, hangDown, slidesUp, slidesDown, slowSlidesUp, slowSlidesDown, slideStop;
    private boolean slidesUpLow, slidesUpMid, slidesUpHigh, boxTouchingSensor;
    private boolean boxOut, boxIn, boxOpen, boxClose, intakeIn, intakeOut, intakeStop;



    //Essentially the main method for the class
    @Override
    public void runOpMode()
    {
        initMotors();
        initSensors();

        waitForStart();

        while (opModeIsActive())
        {
            //Conditions for Controller1/Gamepad1 controls
            stickAngle = Math.toDegrees(Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x));
            forward = gamepad1.left_stick_y > 0.2;
            backward = gamepad1.left_stick_y < -0.2;
            left = gamepad1.left_stick_x > 0.2;
            right = gamepad1.left_stick_x < -0.2;
            diagonal45 = stickAngle > 35 && stickAngle < 55 && gamepad1.left_stick_x > 0;
            diagonal135 = stickAngle > -55 && stickAngle < -35 && gamepad1.left_stick_x < 0;
            diagonal225 = stickAngle > 35 && stickAngle < 55 && gamepad1.left_stick_x < 0;
            diagonal315 = stickAngle > -55 && stickAngle < -35 && gamepad1.left_stick_x > 0;
            slowForward = gamepad1.dpad_up;
            slowBackward = gamepad1.dpad_down;
            slowLeft = gamepad1.dpad_left;
            slowRight = gamepad1.dpad_right;
            turnLeft = gamepad1.right_stick_x < -0.2;
            turnRight = gamepad1.right_stick_x > 0.2;
            quickTurnLeft = gamepad1.left_bumper;
            quickTurnRight = gamepad1.right_bumper;
            stopDrive = gamepad1.left_trigger > 0.25;
            actuatorUp = gamepad1.right_stick_y > 0.25;
            actuatorDown = gamepad1.right_stick_y < -0.25;
            actuatorOff = gamepad1.right_trigger > 0.25;
            launch = gamepad1.x;
            hangUp = gamepad1.y;
            hangDown = gamepad1.a;

            //Conditions for Controller2/Gamepad2 controls
            slidesDown = gamepad2.left_bumper;
            slowSlidesUp = gamepad2.left_stick_y > 0.2;
            slowSlidesDown = gamepad2.left_stick_y < -0.2;
            slidesUpHigh = gamepad2.dpad_up;
            slidesUpMid = gamepad2.dpad_right;
            slidesUpLow = gamepad2.dpad_down;
            slideStop = gamepad2.left_trigger > 0.25;
            boxTouchingSensor = touchSensor.isPressed();
            boxOut = gamepad2.y;
            boxIn = gamepad2.a;
            boxOpen = gamepad2.x;
            boxClose = gamepad2.b;
            intakeIn = gamepad2.right_stick_y < -0.1;
            intakeOut = gamepad2.right_stick_y > 0.1;
            intakeStop = gamepad2.right_trigger > 0.25;


            if (stopDrive) {setMotorsPower(0,0,0,0);}
            else if (forward) {setMotorsPower(pow,pow,pow,pow);}
            else if (backward) {setMotorsPower(-1 * pow, -1 * pow, -1 * pow, -1 * pow);}
            else if (left) {setMotorsPower(-1 * pow, pow,-1 * pow, pow);}
            else if (right) {setMotorsPower(pow,-1 * pow,pow,-1 * pow);}
            else if (diagonal45) {setMotorsPower(0, pow, 0, pow);}
            else if (diagonal135) {setMotorsPower(pow, 0, pow, 0);}
            else if (diagonal225) {setMotorsPower(0, -1 * pow, 0, -1 * pow);}
            else if (diagonal315) {setMotorsPower(-1 * pow, 0, -1 * pow, 0);}
            else if (slowForward) {setMotorsPower(slowPow, slowPow, slowPow, slowPow);}
            else if (slowBackward) {setMotorsPower(-1 * slowPow, -1 * slowPow, -1 * slowPow, -1 * slowPow);}
            else if (slowLeft) {setMotorsPower(-1 * slowPow, slowPow, -1 * slowPow, slowPow);}
            else if (slowRight) {setMotorsPower(slowPow, -1 * slowPow, slowPow, -1 * slowPow);}
            else if (turnLeft) {setMotorsPower(pow,pow,-1 * pow,-1 * pow);}
            else if (turnRight) {setMotorsPower(-1 * pow, -1 * pow, pow, pow);}
            else if (quickTurnLeft) {clockwise(45);}
            else if (quickTurnRight) {counterclockwise(45);}
            else {power(0);}

            if (hangUp) {hanger.setPosition(HANGER_RANGE); gamepad1.rumble(500);}
            else if (hangDown) {hanger.setPosition(HANGER_HOME); gamepad1.rumble(500);}

            if (launch) {launcher.setPosition(LAUNCHER_RANGE); gamepad1.rumble(500);}

            if (actuatorOff) {actuator.setPower(0);}
            else if (actuatorUp)
            {
                actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                actuator.setTargetPosition((int)((32.8 / ((0.5) * Math.PI)) * 384.5));
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(actuator.isBusy() && !(actuatorOff))
                {
                    actuator.setPower(1);
                    updateControls();
                }
                actuator.setPower(0);
            }
            else if (actuatorDown)
            {
                actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                actuator.setTargetPosition((int)((-32.8 / ((0.5) * Math.PI)) * 384.5));
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(actuator.isBusy() && !(actuatorOff))
                {
                    actuator.setPower(-1);
                    updateControls();
                }
                actuator.setPower(0);
            }

            if (slidesDown) {slidesDown();}
            else if (slowSlidesUp) {slides.setPower(0.4);}
            else if (slowSlidesDown) {slides.setPower(-0.3);}
            else if (slidesUpHigh) {slidesUp(3);}
            else if (slidesUpMid) {slidesUp(2);}
            else if (slidesUpLow) {slidesUp(1);}
            else if (boxTouchingSensor || slideStop) {slides.setPower(0); telemetry.addData("Touching", "True"); telemetry.update();}

            if (intakeStop) {intake.setPower(0);}
            else if (intakeIn) {intake.setPower(intakePow);}
            else if (intakeOut) {intake.setPower(-1 * intakePow);}
            else {intake.setPower(0);}

            if (boxOut) {boxOut(); gamepad1.rumble(500);}
            else if (boxIn) {boxIn();}

            if (boxOpen) {openBox(); gamepad2.rumble(250);}
            else if (boxClose) {closeBox(); gamepad2.rumble(250);}

            telemetry.update();
        }
    }

    public void setMotorsPower(double FrontLeft,double BackLeft, double BackRight, double FrontRight)
    {
        frontleft.setPower(FrontLeft);
        backleft.setPower(BackLeft);
        backright.setPower(BackRight);
        frontright.setPower(FrontRight);
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
        frontleft.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection(DcMotorEx.Direction.REVERSE);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);

        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slides.setDirection(DcMotorEx.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        actuator = hardwareMap.get(DcMotorEx.class, "actuator");
        actuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        actuator.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        actuator.setDirection(DcMotorEx.Direction.REVERSE);

        hanger = hardwareMap.get(Servo.class, "hanger");
        hanger.setDirection(Servo.Direction.FORWARD);
        hanger.setPosition(HANGER_HOME);

        launcher = hardwareMap.get(Servo.class, "launcher");
        launcher.setDirection(Servo.Direction.FORWARD);
        launcher.setPosition(LAUNCHER_HOME);

        boxTurner = hardwareMap.get(Servo.class, "box turner");
        boxTurner.setDirection(Servo.Direction.FORWARD);
        boxTurner.setPosition(BOX_TURNER_HOME);

        boxOpener = hardwareMap.get(Servo.class, "box opener");
        boxOpener.setDirection(Servo.Direction.FORWARD);
        boxOpener.setPosition(BOX_OPENER_HOME);
    }

    public void initSensors()
    {
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.XZY, AngleUnit.DEGREES)));
        imu.initialize(parameters);
        currentAngle = 0.0;
        touchSensor = hardwareMap.get(TouchSensor.class, "touch sensor");
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
        double fR = Math.abs(((double)frontright.getCurrentPosition() / frontright.getTargetPosition()));
        double bL = Math.abs((double)backleft.getCurrentPosition() / backleft.getTargetPosition());
        double bR = Math.abs(((double)backright.getCurrentPosition() / backright.getTargetPosition()));
        return ((fL + fR + bL + bR) / 4);
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
        frontright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Forward", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition())) / 4.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5 && opModeIsActive())
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition())) / 4.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
            updateControls();
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(frontright.getTargetPosition()) + Math.abs(backleft.getTargetPosition()) + Math.abs(backright.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition()));
        error /= 4.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Forward", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(frontright.getTargetPosition()) + Math.abs(backleft.getTargetPosition()) + Math.abs(backright.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition()));
            error /= 4.0;
            error = (error / TICKS_PER_REV) * Math.PI;
            updateControls();
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
        frontright.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backright.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Backward", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition())) / 4.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5)
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition())) / 4.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
            updateControls();
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(frontright.getTargetPosition()) + Math.abs(backleft.getTargetPosition()) + Math.abs(backright.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition()));
        error /= 4.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Backward", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(frontright.getTargetPosition()) + Math.abs(backleft.getTargetPosition()) + Math.abs(backright.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition()));
            error /= 4.0;
            error = (error / TICKS_PER_REV) * Math.PI;
            updateControls();
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
        frontright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backright.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Left", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition())) / 4.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5)
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition())) / 4.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
            updateControls();
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(frontright.getTargetPosition()) + Math.abs(backleft.getTargetPosition()) + Math.abs(backright.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition()));
        error /= 4.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Left", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(frontright.getTargetPosition()) + Math.abs(backleft.getTargetPosition()) + Math.abs(backright.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition()));
            error /= 4.0;
            error = (error / TICKS_PER_REV) * Math.PI;
            updateControls();
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


        // Cut off power
        power(0.0);
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
        frontright.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Right", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition())) / 4.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5)
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition())) / 4.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
            updateControls();
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(frontright.getTargetPosition()) + Math.abs(backleft.getTargetPosition()) + Math.abs(backright.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition()));
        error /= 4.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Right", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(frontright.getTargetPosition()) + Math.abs(backleft.getTargetPosition()) + Math.abs(backright.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(frontright.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()) + Math.abs(backright.getCurrentPosition()));
            error /= 4.0;
            error = (error / TICKS_PER_REV) * Math.PI;
            updateControls();
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


        // Cut off power
        power(0.0);
        sleep(50);
        xPos += distance;
    }

    public void resetAngle()
    {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        currentAngle = 0.0;
    }

    public double getAngle()
    {
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        double deltaTheta = orientation.firstAngle - lastAngles.firstAngle;
        if (deltaTheta > 180) {deltaTheta -= 360;}
        else if (deltaTheta <= -180) {deltaTheta += 360;}
        currentAngle += deltaTheta;
        lastAngles = orientation;
        return currentAngle;
    }

    public void clockwise(double degrees)
    {
        resetAngle();
        double error = degrees;
        double pow = LethalCompanyIRL.pow;
        while (Math.abs(error) > 15 && opModeIsActive())
        {
            if (error < 0) {power(pow, -1 * pow, pow, -1 * pow);}
            else {power(-1* pow, pow, -1 * pow, pow);}
            error = degrees - getAngle();
            updateControls();
        }
        while (Math.abs(error) > 2 && opModeIsActive())
        {
            pow = acceleratorTransform(error);
            if (error < 0) {power(pow, -1 * pow, pow, -1 * pow);}
            else {power(-1* pow, pow, -1 * pow, pow);}
            error = degrees - getAngle();
            updateControls();
        }
        power(0);
    }

    public void counterclockwise(double degrees)
    {
        resetAngle();
        double pow = LethalCompanyIRL.pow;
        double error = degrees;
        while (Math.abs(error) > 15 && opModeIsActive())
        {
            if (error < 0) {power(-1 * pow, pow, -1 * pow, pow);}
            else {power(pow, -1 * pow, pow, -1 * pow);}
            error = degrees - getAngle();
            updateControls();
        }
        while (Math.abs(error) > 2 && opModeIsActive())
        {
            pow = acceleratorTransform(error);
            if (error < 0) {power(-1 * pow, pow, -1 * pow, pow);}
            else {power(pow, -1 * pow, pow, -1 * pow);}
            error = degrees - getAngle();
            updateControls();
        }
        power(0);
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
            updateControls();
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
        telemetry.addData("Touching", "True"); telemetry.update();
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

    public void updateControls()
    {
        //Conditions for Controller1/Gamepad1 controls
        stickAngle = Math.toDegrees(Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x));
        forward = gamepad1.left_stick_y > 0.2;
        backward = gamepad1.left_stick_y < -0.2;
        left = gamepad1.left_stick_x > 0.2;
        right = gamepad1.left_stick_x < -0.2;
        diagonal45 = stickAngle > 35 && stickAngle < 55 && gamepad1.left_stick_x > 0;
        diagonal135 = stickAngle > -55 && stickAngle < -35 && gamepad1.left_stick_x < 0;
        diagonal225 = stickAngle > 35 && stickAngle < 55 && gamepad1.left_stick_x < 0;
        diagonal315 = stickAngle > -55 && stickAngle < -35 && gamepad1.left_stick_x > 0;
        slowForward = gamepad1.dpad_up;
        slowBackward = gamepad1.dpad_down;
        slowLeft = gamepad1.dpad_left;
        slowRight = gamepad1.dpad_right;
        turnLeft = gamepad1.right_stick_x < -0.2;
        turnRight = gamepad1.right_stick_x > 0.2;
        quickTurnLeft = gamepad1.left_bumper;
        quickTurnRight = gamepad1.right_bumper;
        stopDrive = gamepad1.left_trigger > 0.25;
        actuatorUp = gamepad1.right_stick_y > 0.25;
        actuatorDown = gamepad1.right_stick_y < -0.25;
        actuatorOff = gamepad1.right_trigger > 0.25;
        launch = gamepad1.x;
        hangUp = gamepad1.y;
        hangDown = gamepad1.a;

        //Conditions for Controller2/Gamepad2 controls
        slidesDown = gamepad2.left_bumper;
        slowSlidesUp = gamepad2.left_stick_y > 0.2;
        slowSlidesDown = gamepad2.left_stick_y < -0.2;
        slidesUpHigh = gamepad2.dpad_up;
        slidesUpMid = gamepad2.dpad_right;
        slidesUpLow = gamepad2.dpad_down;
        slideStop = gamepad2.left_trigger > 0.25;
        boxTouchingSensor = touchSensor.isPressed();
        boxOut = gamepad2.y;
        boxIn = gamepad2.a;
        boxOpen = gamepad2.x;
        boxClose = gamepad2.b;
        intakeIn = gamepad2.right_stick_y < -0.1;
        intakeOut = gamepad2.right_stick_y > 0.1;
        intakeStop = gamepad2.right_trigger > 0.25;


        if (stopDrive) {setMotorsPower(0,0,0,0);}
        else if (forward) {setMotorsPower(pow,pow,pow,pow);}
        else if (backward) {setMotorsPower(-1 * pow, -1 * pow, -1 * pow, -1 * pow);}
        else if (left) {setMotorsPower(-1 * pow, pow,-1 * pow, pow);}
        else if (right) {setMotorsPower(pow,-1 * pow,pow,-1 * pow);}
        else if (diagonal45) {setMotorsPower(0, pow, 0, pow);}
        else if (diagonal135) {setMotorsPower(pow, 0, pow, 0);}
        else if (diagonal225) {setMotorsPower(0, -1 * pow, 0, -1 * pow);}
        else if (diagonal315) {setMotorsPower(-1 * pow, 0, -1 * pow, 0);}
        else if (slowForward) {setMotorsPower(slowPow, slowPow, slowPow, slowPow);}
        else if (slowBackward) {setMotorsPower(-1 * slowPow, -1 * slowPow, -1 * slowPow, -1 * slowPow);}
        else if (slowLeft) {setMotorsPower(-1 * slowPow, slowPow, -1 * slowPow, slowPow);}
        else if (slowRight) {setMotorsPower(slowPow, -1 * slowPow, slowPow, -1 * slowPow);}
        else if (turnLeft) {setMotorsPower(pow,pow,-1 * pow,-1 * pow);}
        else if (turnRight) {setMotorsPower(-1 * pow, -1 * pow, pow, pow);}
        else if (quickTurnLeft) {clockwise(45);}
        else if (quickTurnRight) {counterclockwise(45);}
        else {power(0);}

        if (hangUp) {hanger.setPosition(HANGER_RANGE); gamepad1.rumble(500);}
        else if (hangDown) {hanger.setPosition(HANGER_HOME); gamepad1.rumble(500);}

        if (launch) {launcher.setPosition(LAUNCHER_RANGE); gamepad1.rumble(500);}

        if (actuatorOff) {actuator.setPower(0);}

        if (slidesDown) {slidesDown();}
        else if (slowSlidesUp) {slides.setPower(0.4);}
        else if (slowSlidesDown) {slides.setPower(-0.3);}
        else if (slidesUpHigh) {slidesUp(3);}
        else if (slidesUpMid) {slidesUp(2);}
        else if (slidesUpLow) {slidesUp(1);}
        else if (boxTouchingSensor || slideStop) {slides.setPower(0); telemetry.addData("Touching", "True"); telemetry.update();}

        if (intakeStop) {intake.setPower(0);}
        else if (intakeIn) {intake.setPower(intakePow);}
        else if (intakeOut) {intake.setPower(-1 * intakePow);}
        else {intake.setPower(0);}

        if (boxOut) {boxOut(); gamepad1.rumble(500);}
        else if (boxIn) {boxIn();}

        if (boxOpen) {openBox(); gamepad2.rumble(250);}
        else if (boxClose) {closeBox(); gamepad2.rumble(250);}

        telemetry.update();
    }
}
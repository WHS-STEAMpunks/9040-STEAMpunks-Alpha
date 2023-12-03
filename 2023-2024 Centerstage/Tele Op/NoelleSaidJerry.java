package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Assigns the name for this code that will appear in the driver hub
@TeleOp(name = "Noelle Said \"Jerry\"")

public class NoelleSaidJerry extends LinearOpMode {

    //Declares all DcMotors and Servos
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
    private final static double HANGER_HOME = 0.6;
    private final static double HANGER_RANGE = 0.01;
    private final static double LAUNCHER_HOME = 0.5;
    private final static double LAUNCHER_RANGE = 0.11;
    private final static double BOX_TURNER_HOME = 0.5;
    private final static double BOX_TURNER_RANGE = 0.28;
    private final static double BOX_OPENER_HOME = 0.5;
    private final static double BOX_OPENER_RANGE = 0.108;

    //Declares a timer;
    private ElapsedTime timer;


    //Essentially the main method for the class
    @Override
    public void runOpMode() {

        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        //frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        frontleft.setDirection(DcMotorEx.Direction.REVERSE);

        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        //backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        backleft.setDirection(DcMotorEx.Direction.REVERSE);

        backright = hardwareMap.get(DcMotorEx.class, "back right");
        //backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        backright.setDirection(DcMotorEx.Direction.REVERSE);

        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        //frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        frontright.setDirection(DcMotorEx.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        actuator = hardwareMap.get(DcMotorEx.class, "actuator");
        actuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        actuator.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        actuator.setDirection(DcMotorEx.Direction.FORWARD);

        slides = hardwareMap.get(DcMotorEx.class, "slides");
        //actuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //actuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slides.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        slides.setDirection(DcMotorEx.Direction.FORWARD);

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


        boolean speedMode = true;
        double pow = 0.75;
        boolean slowMode = false;
        timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {


            //Conditions for Controller1/Gamepad1 controls
            boolean forward = gamepad1.left_stick_y > 0.1 || gamepad1.dpad_up;
            boolean backward = gamepad1.left_stick_y < -0.1 || gamepad1.dpad_down;
            boolean left = gamepad1.left_stick_x > 0.1 || gamepad1.dpad_left;
            boolean right = gamepad1.left_stick_x < -0.1 || gamepad1.dpad_right;
            boolean turnLeft = gamepad1.right_stick_x < -0.1;
            boolean turnRight = gamepad1.right_stick_x > 0.1;
            boolean stopDrive = gamepad1.left_trigger > 0.25;
            boolean turnleft180 = gamepad1.left_bumper;
            boolean turnright180 = gamepad1.right_bumper;
            boolean actuatorOff = gamepad1.right_stick_y < -0.25;
            //boolean decelerate = gamepad1.a;
            if (gamepad1.b) {slowMode = !(slowMode);}
            boolean fastMode = false;
            boolean launch = gamepad1.x;
            boolean hangUp = gamepad1.y;
            boolean hangDown = gamepad1.a;

            //Conditions for Controller2/Gamepad2 controls
            boolean up = gamepad2.dpad_up;
            boolean down = gamepad2.dpad_down;
            boolean stay = gamepad2.left_trigger>0.25;
            boolean slideStop = gamepad2.left_bumper;
            boolean boxOut = gamepad2.y;
            boolean boxIn = gamepad2.a;
            boolean boxOpen = gamepad2.x;
            boolean boxClose = gamepad2.b;
            boolean intakeIn = gamepad2.left_stick_y < -0.1;
            boolean intakeOut = gamepad2.left_stick_y > 0.1;
            boolean intakeStop = gamepad2.right_bumper;
            boolean actuatorUp = gamepad2.right_stick_y > 0.25;
            boolean actuatorDown = gamepad2.right_stick_y < -0.25;

            //Conditions for stopping all movements
            boolean isZero = true;


            if (slowMode) {pow = 0.3;}
            else {pow = 0.75;}

            if(stopDrive){
                isZero = false;
                setMotorsPower(0,0,0,0);
            }
            //--------------------------------------Forward
            else if(forward){
                isZero = false;
                setMotorsPower(pow,pow,pow,pow);
            }
            //--------------------------------------Backward
            else if(backward){
                isZero = false;
                setMotorsPower(-pow, -pow, -pow, -pow);
            }
            //--------------------------------------Left
            else if(left){
                isZero = false;
                setMotorsPower(-pow,pow,-pow,pow);
            }
            //--------------------------------------Right
            else if(right){
                isZero = false;
                setMotorsPower(pow,-pow,pow,-pow);
            }
            //--------------------------------------TurnLeft
            else if(turnLeft){
                isZero = false;
                setMotorsPower(pow,pow,-pow,-pow);
            }
            //--------------------------------------TurnRight
            else if(turnRight){
                isZero = false;
                setMotorsPower(-pow,-pow,pow,pow);
            }
            //--------------------------------------TurnLeft180
            else if(turnleft180){
                isZero = false;
                timer.reset();
                while (timer.milliseconds() < 600)
                {
                    setMotorsPower(1,1,-1,-1);

                }
                setMotorsPower(0, 0, 0, 0);
                speedMode = true;
            }
            //--------------------------------------TurnRight180
            else if(turnright180){
                isZero = false;
                timer.reset();
                while (timer.milliseconds() < 600)
                {
                    setMotorsPower(-1,-1,1,1);
                }
                setMotorsPower(0, 0, 0, 0);
                speedMode = true;
            }
            else if (slowMode)
            {
                setMotorsPower(0, 0, 0, 0);
            }

            if (hangUp)
            {
                hanger.setPosition(HANGER_RANGE);
            }
            else if (hangDown)
            {
                hanger.setPosition(HANGER_HOME);
            }
            if (launch)
            {
                launcher.setPosition(LAUNCHER_RANGE);
            }

            if (up)
            {
                slides.setPower(0.8);
            }
            else if (down)
            {
                slides.setPower(-0.5);
            }
            else if (stay)
            {
                slides.setPower(0.1);
            }
            else if (slideStop)
            {
                slides.setPower(0);
            }
            if (intakeIn)
            {
                isZero = false;
                intake.setPower(0.75);
            }
            else if (intakeOut)
            {
                isZero = false;
                intake.setPower(-0.75);
            }
            else if(intakeStop)
            {
                isZero = false;
                intake.setPower(0);
            }
            if (boxOut)
            {
                boxTurner.setPosition(BOX_TURNER_RANGE);
            }
            else if (boxIn)
            {
                boxTurner.setPosition(BOX_TURNER_HOME);
            }
            if (boxOpen)
            {
                boxOpener.setPosition(BOX_OPENER_RANGE);
            }
            else if (boxClose)
            {
                boxOpener.setPosition(BOX_OPENER_HOME);
            }
            if (actuatorUp)
            {
                actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                actuator.setTargetPosition((int)((-5.64 / ((0.5) * Math.PI)) * 537.7));
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(actuator.isBusy() && !(actuatorOff)) {actuator.setPower(-1);}
                actuator.setPower(0);
            }
            else if (actuatorDown)
            {
                actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                actuator.setTargetPosition((int)((5.6 / ((0.5) * Math.PI)) * 537.7));
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(actuator.isBusy() && !(actuatorOff)) {actuator.setPower(1);}
                actuator.setPower(0);
            }
            if (actuatorOff) {actuator.setPower(0);}


            telemetry.update();

        }
        launcher.setPosition(LAUNCHER_HOME);
    }

    public void setMotorsPower(double FrontLeft,double BackLeft, double BackRight, double FrontRight)
    {
        frontleft.setPower((-1*FrontLeft));
        backleft.setPower((-1*BackLeft));
        backright.setPower(BackRight);
        frontright.setPower(FrontRight);
    }
}
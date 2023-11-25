package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Assigns the name for this code that will appear in the driver hub
@TeleOp(name = "thiscodeisbroken")

public class thiscodeisbroken extends LinearOpMode {

    //Declares all DcMotors and Servos
    private DcMotorEx frontleft;
    private DcMotorEx backright;
    private DcMotorEx backleft;
    private DcMotorEx frontright;
    private DcMotorEx slide1;
    private DcMotorEx slide2;
    private DcMotorEx intake;
    private Servo box1;
    private Servo box2;
    private final static double BOX1_HOME = 0.5;
    private final static double BOX1_MAX_RANGE = 0.118;

    private final double BOX2_HOME = 0.135;
    private final double BOX2_MAX_RANGE = 0.118;

    //Declares a timer;
    private ElapsedTime timer;

    //Method to set motor power for all drivetrain motors
    public void setMotorsPower(double FrontLeft,double BackLeft, double BackRight, double FrontRight)
    {
        frontleft.setPower((-1*FrontLeft));
        backleft.setPower((-1*BackLeft));
        backright.setPower(BackRight);
        frontright.setPower(FrontRight);
    }
    public void boxOut()
    {
        box1.setPosition(BOX1_MAX_RANGE);
        box2.setPosition(BOX2_MAX_RANGE);
    }
    public void boxIn()
    {
        box1.setPosition(BOX1_HOME);
        box2.setPosition(BOX2_HOME);
    }

    //Essentially the main method for the class
    @Override
    public void runOpMode() {

        //Maps the DcMotors and Servos to their respective names stated in the driver hub
        
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        //frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        //backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backright = hardwareMap.get(DcMotorEx.class, "back right");
        //backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        //frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        
        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        //slide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slide1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide1.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        //slide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slide2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide2.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        box1 = hardwareMap.get(Servo.class, "box1");
        box1.setDirection(Servo.Direction.FORWARD);
        box1.setPosition(BOX1_HOME);

        box2 = hardwareMap.get(Servo.class, "box2");
        box2.setDirection(Servo.Direction.FORWARD);
        box2.setPosition(BOX2_HOME);


        boolean speedMode = true;
        double pow;
        timer = new ElapsedTime();
        pow = 1;
        waitForStart();

        while (opModeIsActive()) {

            
            //Conditions for Controller1/Gamepad1 controls
            boolean forward = gamepad1.left_stick_y > 0.1;
            boolean backward = gamepad1.left_stick_y < -0.1;
            boolean left = gamepad1.right_stick_x > 0.1;
            boolean right = gamepad1.right_stick_x < -0.1;
            boolean strafeLeft = gamepad1.left_bumper;
            boolean strafeRight = gamepad1.right_bumper;
            boolean stop = gamepad1.y;
            boolean turnleft180 = gamepad1.dpad_left;
            boolean turnright180 = gamepad1.dpad_right;
            boolean decelerate = gamepad1.a;
            boolean slowMode = gamepad1.left_trigger > 0.25;
            boolean fastMode = gamepad1.x;

            //Conditions for Controller2/Gamepad2 controls
            boolean up = gamepad2.right_trigger>0.25;
            boolean down = gamepad2.dpad_down;
            boolean stay = gamepad2.left_trigger>0.25;
            boolean slideStop = gamepad2.left_bumper;
            boolean boxOut = gamepad2.y;
            boolean boxIn = gamepad2.a;
            boolean intakeIn = gamepad2.left_stick_y < -0.1;
            boolean intakeOut = gamepad2.left_stick_y > 0.1;
            boolean intakeStop = gamepad2.b;

            //Conditions for stopping all movements
            boolean isZero = true;

            //Changes the speed of the movement
            if (slowMode) {
                pow = 1;
            } 
            else if (fastMode)
            {
                pow = 1;
            }
            else {
                pow = 1;
            }


            //--------------------------------------StopBot
            if(stop){
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
                setMotorsPower(pow,pow,-pow,-pow);
            }
            //--------------------------------------Right
            else if(right){
                isZero = false;
                setMotorsPower(-pow,-pow,pow,pow);
            }
            //--------------------------------------StrafeLeft
            else if(strafeLeft){
                isZero = false;
                setMotorsPower(pow,-pow,pow,-pow);
            }
            //--------------------------------------StrafeRight
            else if(strafeRight){
                isZero = false;
                setMotorsPower(-pow,pow,-pow,pow);
            }
            //--------------------------------------Decelerate
            else if (decelerate)
            {
                while (backleft.getVelocity() > 0.5)
                {
                    double state = Math.tanh(0.32 + 0.05 * backleft.getVelocity());
                    backleft.setVelocity(state);
                    backright.setVelocity(state);
                    frontleft.setVelocity(state);
                    frontright.setVelocity(state);

                }
                setMotorsPower(0, 0, 0, 0);
            }
            //--------------------------------------TurnLeft180
            else if(turnleft180){
                isZero = false;
                timer.reset();
                while (timer.milliseconds() < 600)
                {
                    setMotorsPower(-1,-1,1,1);

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
                    setMotorsPower(1,1,-1,-1);
                }
                setMotorsPower(0, 0, 0, 0);
                speedMode = true;
            }
            //--------------------------------------FastRightStrafe
            //--------------------------------------FastLeftStrafe
            //--------------------------------------FastForward
            if(up)
            {
                isZero = false;
                slide1.setPower(0.8);
                slide2.setPower(-0.8);
                telemetry.update();
            }
            else if(down)
            {
                isZero = false;
                slide1.setPower(-0.5);
                slide2.setPower(0.5);
                telemetry.update();
            }
            else if(stay)
            {
                isZero = false;
                slide1.setPower(0.1);
                slide2.setPower(-0.1);
                telemetry.update();
            }
            else if (slideStop)
            {
                slide1.setPower(0);
                slide2.setPower(0);
                telemetry.update();
            }
            if (intakeIn)
            {
                isZero = false;
                intake.setPower(1);
            }
            else if (intakeOut)
            {
                isZero = false;
                intake.setPower(-1);
            }
            else if(intakeStop)
            {
                isZero = false;
                intake.setPower(0);
            }
            if(boxOut)
            {
                isZero = false;
                boxOut();
            }
            else if(boxIn)
            {
                isZero = false;
                boxIn();
            }
            else {
                setMotorsPower(0,0,0,0);
            }


            telemetry.update();

        }
    }
}
//package org.firstinspires.ftc.teamcode;
/*
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private DcMotorEx slide;
    private DcMotorEx intake;
    private Servo box;
    private final static double BOX1_HOME = 0.5;
    private final static double BOX1_MAX_RANGE = 0.118;

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
        box.setPosition(BOX1_MAX_RANGE);
    }
    public void boxIn()
    {
        box.setPosition(BOX1_HOME);
    }

    //Essentially the main method for the class
    @Override
    public void runOpMode() {

        //Maps the DcMotors and Servos to their respective names stated in the driver hub

        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        //frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        frontleft.setDirection(DcMotorEx.Direction.FORWARD);

        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        //backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        backleft.setDirection(DcMotorEx.Direction.FORWARD);

        backright = hardwareMap.get(DcMotorEx.class, "back right");
        //backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        backright.setDirection(DcMotorEx.Direction.FORWARD);

        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        //frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        frontright.setDirection(DcMotorEx.Direction.FORWARD);

        slide = hardwareMap.get(DcMotorEx.class, "slide");
        //slide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slide1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        slide.setDirection(DcMotorEx.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        box = hardwareMap.get(Servo.class, "box1");
        box.setDirection(Servo.Direction.FORWARD);
        box.setPosition(BOX1_HOME);


        boolean speedMode = true;
        double pow = 1;
        boolean slowMode = false;
        timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {


            //Conditions for Controller1/Gamepad1 controls
            boolean forward = gamepad1.left_stick_y > 0.1 || gamepad1.dpad_up;
            boolean backward = gamepad1.left_stick_y < -0.1 || gamepad1.dpad_down;
            boolean left = gamepad1.left_stick_x > 0.1 || gamepad1.dpad_left;
            boolean right = gamepad1.left_stick_x < -0.1 || gamepad1.dpad_right;
            boolean turnLeft = gamepad1.left_bumper;
            boolean turnRight = gamepad1.right_bumper;
            boolean stopDrive = gamepad1.left_trigger > 0.25;
            boolean turnleft180 = gamepad1.dpad_left;
            boolean turnright180 = gamepad1.dpad_right;
            //boolean decelerate = gamepad1.a;
            if (gamepad1.y) {slowMode = !(slowMode);}
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


            if (slowMode) {pow = 0.3;}
            else {pow = 1;}

            //--------------------------------------StopBot
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
                setMotorsPower(pow,pow,-pow,-pow);
            }
            //--------------------------------------Right
            else if(right){
                isZero = false;
                setMotorsPower(-pow,-pow,pow,pow);
            }
            //--------------------------------------TurnLeft
            else if(turnLeft){
                isZero = false;
                setMotorsPower(-pow,-pow,pow,pow);
            }
            //--------------------------------------TurnRight
            else if(turnRight){
                isZero = false;
                setMotorsPower(pow,pow,-pow,-pow);
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
                slide.setPower(0.8);
                telemetry.update();
            }
            else if(down)
            {
                isZero = false;
                slide.setPower(-0.5);
                telemetry.update();
            }
            else if(stay)
            {
                isZero = false;
                slide.setPower(0.1);
                telemetry.update();
            }
            else if (slideStop)
            {
                slide.setPower(0);
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


            telemetry.update();

        }
    }
}*/
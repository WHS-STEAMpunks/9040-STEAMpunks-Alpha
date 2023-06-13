package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

//Assigns the name for this code that will appear in the driver hub
@TeleOp(name = "DriverV2")

public class TeleOPtest extends LinearOpMode {

    //Declares all DcMotors and Servos
    private DcMotorEx frontleft, backright, backleft, frontright;

    //Declares a timer
    private ElapsedTime timer;


    //Method to set motor power for all drivetrain motors
    private void setMotorsPower(double FrontLeft, double BackLeft, double BackRight, double FrontRight) {
        frontleft.setPower(FrontLeft);
        backleft.setPower(BackLeft);
        backright.setPower(BackRight);
        frontright.setPower(FrontRight);
    }
    public void setMotorsPower(double pow){
        frontleft.setPower(pow);
        backleft.setPower(pow);
        backright.setPower(pow);
        frontright.setPower(pow);
    }


    //Essentially the main method for the class
    @Override
    public void runOpMode() {

        //Maps the DcMotors their respective names stated in the driver hub
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        backright = hardwareMap.get(DcMotorEx.class, "back right");
        frontright = hardwareMap.get(DcMotorEx.class, "front right");

        //creates a new ElapsedTime object for the timer
        timer = new ElapsedTime();

        boolean speedMode = true;
        double pow;

        waitForStart();

        boolean forward, backward, left, right, strafeLeft, strafeRight, turnLeft180, turnRight180, slowMode, stop;

        while (opModeIsActive()) {

            //Conditions for Controller1/Gamepad1 controls
            forward = gamepad1.left_stick_y > 0.1;
            backward = gamepad1.left_stick_y < -0.1;
            left = gamepad1.right_stick_x > 0.1;
            right = gamepad1.right_stick_x < -0.1;
            strafeLeft = gamepad1.left_bumper;
            strafeRight = gamepad1.right_bumper;
            turnLeft180 = gamepad1.dpad_left;
            turnRight180 = gamepad1.dpad_right;
            slowMode = gamepad1.left_trigger > 0.25;
            stop = gamepad1.y;

            //Changes the speed of the movement
            if (speedMode) {
                pow = 0.47;
            } else if (slowMode) {
                pow = 0.35;
            } else {
                pow = 0.47;
            }

            //--------------------------------------------------------------------StopBot
            if (stop) {
                setMotorsPower(0);
            }
            //--------------------------------------------------------------------DiagonalLeftForward
            else if (forward && strafeLeft) {
                setMotorsPower(pow);
                setMotorsPower(pow, -pow, pow, -pow);
            }
            //--------------------------------------------------------------------DiagonalRightForward
            else if (forward && strafeRight) {
                setMotorsPower(pow);
                setMotorsPower(-pow, pow, -pow, pow);
            }
            //--------------------------------------------------------------------DiagonalLeftBackward
            else if (backward && strafeLeft) {
                setMotorsPower(-pow);
                setMotorsPower(pow, -pow, pow, -pow);
            }
            //--------------------------------------------------------------------DiagonalRightForward
            else if (backward && strafeRight) {
                setMotorsPower(-pow);
                setMotorsPower(-pow, pow, -pow, pow);
            }
            //--------------------------------------------------------------------Forward
            else if (forward) {
                setMotorsPower(pow);
            }
            //--------------------------------------------------------------------Backward
            else if (backward) {
                setMotorsPower(-pow);
            }
            //--------------------------------------------------------------------Left
            else if (left) {
                setMotorsPower(-pow, -pow, pow, pow);
            }
            //--------------------------------------------------------------------Right
            else if (right) {
                setMotorsPower(pow, pow, -pow, -pow);
            }
            //--------------------------------------------------------------------StrafeLeft
            else if (strafeLeft) {
                setMotorsPower(pow, -pow, pow, -pow);
            }
            //--------------------------------------------------------------------StrafeRight
            else if (strafeRight) {
                setMotorsPower(-pow, pow, -pow, pow);
            }
            //--------------------------------------------------------------------TurnLeft180
            else if (turnLeft180) {
                timer.reset();
                while (timer.milliseconds() < 600) {
                    setMotorsPower(-1, -1, 1, 1);
                }
                setMotorsPower(0);
                speedMode = true;
            }
            //--------------------------------------------------------------------TurnRight180
            else if (turnRight180) {
                timer.reset();
                while (timer.milliseconds() < 600) {
                    setMotorsPower(1, 1, -1, -1);
                }
                setMotorsPower( 0);
                speedMode = true;
            }
            //--------------------------------------------------------------------Decelerate
            else {
                while (backleft.getVelocity() > 0.5) {
                    double state = Math.tanh(0.32 + 0.05 * backleft.getVelocity());
                    backleft.setVelocity(state);
                    backright.setVelocity(state);
                    frontleft.setVelocity(state);
                    frontright.setVelocity(state);
                }
                setMotorsPower(0);
            }

            telemetry.update();
        }
    }
}

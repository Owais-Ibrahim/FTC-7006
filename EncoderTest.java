package org.firstinspires.ftc.teamcode.TESTING;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwarePushbot;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="EncoderTest", group="Pushbot")
@Disabled
public class EncoderTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.858;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        telemetry.addLine("Robot is ready to start");
        telemetry.update();

        EncoderReset();
        GyroCalibirate();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();




        EncodedFrontDrive(10,0.5);
        sleep(100);
        TurnAbsolute(0,0.4,2);
        sleep(100);
        EncodedBackDrive(10,0.5);
        sleep(100);
        TurnAbsolute(0,0.4,2);
        sleep(100);
        EncodedStrafeRight(10,0.5);
        sleep(100);
        TurnAbsolute(0,0.4,2);
        sleep(100);
        EncodedStrafeLeft(10,0.5);
        sleep(100);
        TurnAbsolute(0,0.4,2);
        sleep(100);
        EncodedRightDiagonal(10,0.5);
        sleep(100);
        EncodedLeftDiagonal(10,0.5);
        sleep(100);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void EncodedLeftDiagonal(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.rightFront.setTargetPosition((RotationsNeeded)+(robot.rightFront.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.update();

        // Turn On RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightFront.setPower(speed);
        robot.leftBack.setPower(speed);

        while (robot.rightFront.isBusy()) {

            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(-speed);

            // Display it for the driver.
            telemetry.addData("Right:", robot.rightFront.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void EncodedRightDiagonal(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.leftFront.setTargetPosition((-RotationsNeeded)+(robot.leftFront.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.update();

        // Turn On RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setPower(speed);
        robot.rightBack.setPower(speed);

        while (robot.leftFront.isBusy()) {

            robot.leftFront.setPower(speed);
            robot.rightBack.setPower(speed);

            // Display it for the driver.
            telemetry.addData("Right:", robot.leftFront.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void EncodedFrontDrive(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        robot.rightFront.setTargetPosition((RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.leftFront.setTargetPosition((-RotationsNeeded)+(robot.leftFront.getCurrentPosition()));

        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.update();

        // Turn On RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(-speed);

        while (robot.rightFront.isBusy()) {

            robot.rightBack.setPower(speed);
            robot.leftBack.setPower(-speed);

            // Display it for the driver.
            telemetry.addData("Right:", robot.rightFront.getCurrentPosition());
            telemetry.update();
            telemetry.addData("Left:", robot.leftFront.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void EncodedBackDrive(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));
        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.update();

        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.leftFront.setTargetPosition((RotationsNeeded)+(robot.leftFront.getCurrentPosition()));

        // Turn On RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(-speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);

        while (robot.rightFront.isBusy()) {

            robot.rightBack.setPower(-speed);
            robot.leftBack.setPower(speed);

            // Display it for the driver.
            telemetry.addData("Right:", robot.rightFront.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void EncodedStrafeRight(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));
        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.update();

        robot.rightFront.setTargetPosition((-RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.leftFront.setTargetPosition((-RotationsNeeded)+(robot.leftFront.getCurrentPosition()));

        // Turn On RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);

        while (robot.rightFront.isBusy()) {

            robot.rightBack.setPower(speed);
            robot.leftBack.setPower(speed);

            // Display it for the driver.
            telemetry.addData("Right:", robot.rightFront.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void EncodedStrafeLeft(double inches, double speed) throws InterruptedException //Make negative distance if want to forward or backward depending on motors
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));
        telemetry.addData("RightFront:", RotationsNeeded);
        telemetry.update();

        robot.rightFront.setTargetPosition((RotationsNeeded)+(robot.rightFront.getCurrentPosition()));
        robot.leftFront.setTargetPosition((RotationsNeeded)+(robot.leftFront.getCurrentPosition()));

        // Turn On RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(-speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(-speed);

        while (robot.rightFront.isBusy()) {

            robot.rightBack.setPower(-speed);
            robot.leftBack.setPower(-speed);

            // Display it for the driver.
            telemetry.addData("Right:", robot.rightFront.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();


        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void EncoderReset()throws InterruptedException
    {
        telemetry.addLine("Status Resetting Encoders");
        telemetry.update();

        //robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitOneFullHardwareCycle();

        //robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitOneFullHardwareCycle();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Left:", "at %2d", robot.rightFront.getCurrentPosition());
        telemetry.update();

    }


    //Gyro
    public void GyroCalibirate() throws InterruptedException
    {
        robot.gyro.calibrate();

        while (robot.gyro.isCalibrating())
        {
            telemetry.addLine("Calibrating");
            telemetry.update();
        }

        telemetry.addLine("Robot Done Calibrating");
        telemetry.update();
    }

    public void TurnAbsolute(int target,double turnspeed, int threshold) throws InterruptedException
    {
        int Heading= robot.gyro.getHeading();
        if (Heading>180)
        {
            Heading= robot.gyro.getHeading()-360;
        }

        while (Math.abs(Heading-target) > threshold)
        {
            if (Heading>180)
            {
                Heading= robot.gyro.getHeading()-360;
            }

            telemetry.addData("CurrentHeading:", Heading);
            telemetry.update();

            if (Heading>target)
            {
                //CW
                robot.rightFront.setPower(turnspeed);
                robot.rightBack.setPower(turnspeed);
                robot.leftFront.setPower(turnspeed);
                robot.leftBack.setPower(turnspeed);

                telemetry.addData("Heading>Target",Heading );
                telemetry.update();

            }
            if (Heading<target)
            {
                //CCW
                robot.rightFront.setPower(-turnspeed);
                robot.rightBack.setPower(-turnspeed);
                robot.leftFront.setPower(-turnspeed);
                robot.leftBack.setPower(-turnspeed);

                telemetry.addData("Heading<Target", Heading );
                telemetry.update();
            }


            Heading=robot.gyro.getHeading();
        }



        waitOneFullHardwareCycle();

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        telemetry.addData("CurrentHeading:", robot.gyro.getHeading());
        telemetry.update();
    }





}



package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name= "Red Block2", group="Sky autonomous")
//@Disabled
public class RedBlock extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();

    private DistanceSensor FrontSensor;
    private DistanceSensor BackSensor=null;


    //Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.858;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     TourqueMotorTicks    = 1440 ;
    static final double     PulleyDiameter   = 1.25;
    static final double     LiftInch         = (TourqueMotorTicks) / (PulleyDiameter * 3.1415);

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.5;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        FrontSensor = hardwareMap.get(DistanceSensor.class, "frontrange");
        BackSensor=hardwareMap.get(Rev2mDistanceSensor.class,"backrange");

        double Frontrange=FrontSensor.getDistance(DistanceUnit.CM);
        double BackRange=BackSensor.getDistance(DistanceUnit.CM);
        double SideRange= robot.SideSensor.cmUltrasonic();


        GyroCalibirate();

        waitForStart();

        FrontSensor(38,0.4);
        //sleep(100);
        gyroTurn(0.3,-86);
        sleep(100);

        double range=BackSensor.getDistance(DistanceUnit.CM);
        double speed=0.4;
        telemetry.addData("range:", range);
        telemetry.update();
        while(range<65)
        {
            robot.rightFront.setPower(-speed);
            robot.rightBack.setPower(-speed);
            robot.leftFront.setPower(speed);
            robot.leftBack.setPower(speed);

            telemetry.addData("range:", range);
            telemetry.update();
            range=BackSensor.getDistance(DistanceUnit.CM);

        }

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);



        SideRange=robot.SideSensor.cmUltrasonic();
        speed=0.3;
        telemetry.addData("range:", SideRange);
        telemetry.update();
        while(SideRange>8)
        {
            robot.rightFront.setPower(-speed);
            robot.rightBack.setPower(speed);
            robot.leftFront.setPower(-speed);
            robot.leftBack.setPower(speed);

            telemetry.addData("range:", range);
            telemetry.update();
            SideRange=robot.SideSensor.cmUltrasonic();

        }

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);

        gyroTurn(0.3,-86);



        robot.AutoArm.setPosition(0);
        sleep(600);
        robot.AutoClaw.setPosition(0);
        sleep(1000);
        robot.AutoArm.setPosition(0.8);
        sleep(200);


        StrafePower(-0.4,600);
        gyroTurn(0.3,-86);



        speed=0.4;
        robot.rightFront.setPower(-speed);
        robot.rightBack.setPower(-speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        sleep(800);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);

        gyroTurn(0.3,-86);

        speed=0.4;
        robot.rightFront.setPower(-speed);
        robot.rightBack.setPower(-speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        sleep(800);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);


        FrontSensor(80,0.4);

        gyroTurn(0.3,-86);


        StrafePower(0.4,700);

        gyroTurn(0.3,-86);

        //StrafePower(0.4,700);


        robot.AutoArm.setPosition(0);
        sleep(500);
        robot.AutoClaw.setPosition(1);
        sleep(600);
        robot.AutoArm.setPosition(0.55);
        sleep(200);
        StrafePower(-0.5,400);

        gyroTurn(0.3,-180);

        robot.GrabFoundationLeft.setPosition(0);
        robot.GrabFoundationRight.setPosition(1);

        //range=FrontSensor.getDistance(DistanceUnit.CM);
        speed=0.2;
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(-speed);
        robot.leftBack.setPower(-speed);
        sleep(1000);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);

        robot.GrabFoundationLeft.setPosition(1);
        robot.GrabFoundationRight.setPosition(0);
        sleep(1000);

        speed=0.2;
        robot.rightFront.setPower(-speed);
        robot.rightBack.setPower(-speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        sleep(3000);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);

        gyroTurn(0.6,-225);

        SideRange=robot.SideSensor.cmUltrasonic();
        speed=0.3;
        telemetry.addData("range:", SideRange);
        telemetry.update();
        while(SideRange>8)
        {
            robot.rightFront.setPower(-speed);
            robot.rightBack.setPower(speed);
            robot.leftFront.setPower(-speed);
            robot.leftBack.setPower(speed);

            telemetry.addData("range:", range);
            telemetry.update();
            SideRange=robot.SideSensor.cmUltrasonic();

        }

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);
        gyroTurn(0.6,-270);

        speed=0.8;

        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(-speed);
        robot.leftBack.setPower(-speed);

        sleep(2000);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);



    }

    //1st Block Code (Based on vision)
    public void GetBlock1() throws InterruptedException
    {


    }

    public void GetBlock2()throws InterruptedException
    {

    }

    public void GetBlock3()throws InterruptedException
    {
        /*
     double  speed=0.4;
        robot.rightFront.setPower(-speed);
        robot.rightBack.setPower(-speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        sleep(900);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);

        FrontSensor(70,0.4);

        SideRange=robot.SideSensor.cmUltrasonic();
        speed=0.3;
        telemetry.addData("range:", SideRange);
        telemetry.update();
        while(SideRange>8)
        {
            robot.rightFront.setPower(-speed);
            robot.rightBack.setPower(speed);
            robot.leftFront.setPower(-speed);
            robot.leftBack.setPower(speed);

            telemetry.addData("range:", range);
            telemetry.update();
            SideRange=robot.SideSensor.cmUltrasonic();

        }

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);

        gyroTurn(0.3,-86);

        robot.AutoArm.setPosition(0);
        sleep(500);
        robot.AutoClaw.setPosition(0);
        sleep(600);
        robot.AutoArm.setPosition(0.8);
        sleep(200);

        StrafePower(-0.4,600);

        sleep(100);

        gyroTurn(0.3,-86);

        speed=0.4;
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(-speed);
        robot.leftBack.setPower(-speed);
        sleep(1000);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);

        gyroTurn(0.3,-86);

        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(-speed);
        robot.leftBack.setPower(-speed);
        sleep(1000);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);

        BackRange=BackSensor.getDistance(DistanceUnit.CM);
        speed=0.4;
        telemetry.addData("range:", range);
        telemetry.update();
        while(BackRange>30)
        {
            robot.rightFront.setPower(speed);
            robot.rightBack.setPower(speed);
            robot.leftFront.setPower(-speed);
            robot.leftBack.setPower(-speed);

            telemetry.addData("range:", range);
            telemetry.update();
            BackRange=BackSensor.getDistance(DistanceUnit.CM);

        }

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);

        StrafePower(0.4,600);

        robot.AutoArm.setPosition(0);
        sleep(500);
        robot.AutoClaw.setPosition(1);
        sleep(600);
        robot.AutoArm.setPosition(0.55);
        sleep(200);
        */

    }
    public void StrafePower(double speed, int sleep)throws InterruptedException
    {
        robot.rightFront.setPower(-speed);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(-speed);
        robot.leftBack.setPower(speed);
        sleep(sleep);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);



    }


    public void FrontSensor(double inches, double speed)
    {
        double range=FrontSensor.getDistance(DistanceUnit.CM);
        while(range>inches)
        {
            robot.rightFront.setPower(-speed);
            robot.rightBack.setPower(-speed);
            robot.leftFront.setPower(speed);
            robot.leftBack.setPower(speed);

            telemetry.addData("range:", range);
            telemetry.update();
            range=FrontSensor.getDistance(DistanceUnit.CM);

        }
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        sleep(200);
    }



    //Encoder
    public void LiftUp (double inches, double speed) throws InterruptedException
    {
        int RotationsNeeded= (int)(inches*(LiftInch));

        robot.Lift.setTargetPosition((-RotationsNeeded)+(robot.Lift.getCurrentPosition()));
        // Turn On RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Lift.setPower(speed);
        while (robot.Lift.isBusy())
        {
            // Display it for the driver.
            telemetry.addData("Lift Up:", robot.Lift.getCurrentPosition());
            telemetry.update();

        }

        waitOneFullHardwareCycle();

        // Stop all motion;
        robot.Lift.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void LiftDown (double inches, double speed) throws InterruptedException
    {
        int RotationsNeeded= (int)(inches*(LiftInch));

        robot.Lift.setTargetPosition((RotationsNeeded)+(robot.Lift.getCurrentPosition()));
        // Turn On RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Lift.setPower(speed);
        while (robot.Lift.isBusy())
        {

            // Display it for the driver.
            telemetry.addData("Lift Up:", robot.Lift.getCurrentPosition());
            telemetry.update();
        }

        waitOneFullHardwareCycle();

        // Stop all motion;
        robot.Lift.setPower(0);
        robot.Push.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }


    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);


            rightSpeed  = speed * steer;
            leftSpeed   = rightSpeed;

        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwarePushbot;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BlueBlock", group="Pushbot")
public class BlueBlock extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
      
        waitForStart();
        
        StrafeRight(0.45,1850); //Gets close to blocks); //Gets close to blocks
        sleep(1000);
        Drive(-.5,475); //Gets to third block
        sleep(100);
        robot.arm.setPosition(1.0); //Brings arm down
        sleep(1000);
        StrafeLeft(0.4,900); //Moves bock left
        sleep(1000);
        Drive(0.4,2900); //Drive across bridge
        sleep(1000);
        robot.arm.setPosition(0); //Move servo up
        sleep(1000);
        Drive(-0.4,2000); //Drive back across bridge
        sleep(1000);
        StrafeRight(0.45,1250); //Goes to 1st block
        sleep(1000);
        robot.arm.setPosition(1.0); //Grabs it
        sleep(1000);
        StrafeLeft(0.5,1400); //Moves it left
        sleep(1000);
        Drive(0.4,1900); //Drives across bridge
        sleep(1000);
        robot.arm.setPosition(0); //Arm up
        sleep(1000);
        Drive(-0.4,700); //Move under bridge
        sleep(1000);
        StrafeRight(0.5,1000); //Clear space 
        sleep(1000);

    }
    public void StrafeLeft (double strafe, int sleep)
    {
        robot.rightFront.setPower(strafe);
        robot.rightBack.setPower(-strafe);
        robot.leftFront.setPower(strafe);
        robot.leftBack.setPower(-strafe);

        sleep(sleep);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        sleep (100);
    }

    public void StrafeRight (double strafe, int sleep)
    {
        robot.rightFront.setPower(-strafe);
        robot.rightBack.setPower(strafe);
        robot.leftFront.setPower(-strafe);
        robot.leftBack.setPower(strafe);

        sleep(sleep);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        sleep (100);
    }

    public void Drive (double drive, int sleep)
    {
        robot.rightFront.setPower(drive);
        robot.rightBack.setPower(drive);
        robot.leftFront.setPower(-drive);
        robot.leftBack.setPower(-drive);

        sleep(sleep);

        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);

        sleep (100);
    }

}
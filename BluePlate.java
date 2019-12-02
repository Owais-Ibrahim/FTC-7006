import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwarePushbot;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BluePlate", group="Pushbot")
public class BluePlate extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
      
        waitForStart();
        
        sleep(20000);
        StrafeLeft(0.5,200);
        sleep(1000);
        Drive(0.5,600);
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
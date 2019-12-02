import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwarePushbot;
import org.firstinspires.ftc.teamcode.HardwarePushbot2;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="EncoderTest", group="Pushbot")
@Disabled
public class EncoderTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot2 robot = new HardwarePushbot2();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;  /*** 3.858 */
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() throws InterruptedException{

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        EncoderReset();

        telemetry.addLine("Robot is ready to start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        EncodedDrive(5,0.4);
        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void EncodedDrive(double inches, double speed) throws InterruptedException
    {
        int RotationsNeeded= (int)(inches*(COUNTS_PER_INCH));

        if(opModeIsActive())
        {
            robot.right.setTargetPosition(RotationsNeeded);
            robot.left.setTargetPosition(-RotationsNeeded);

            // Turn On RUN_TO_POSITION
            robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.right.setPower(speed);
            robot.left.setPower(speed);

            while (robot.right.isBusy() && robot.left.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Right:", robot.right.getCurrentPosition());
                telemetry.addData("Left:", robot.left.getCurrentPosition());
                telemetry.update();

            }

            waitOneFullHardwareCycle();

            // Stop all motion;
            robot.right.setPower(0);
            robot.left.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void EncoderReset()
    {
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Right:", robot.right.getCurrentPosition());
        telemetry.addData("Left:", robot.left.getCurrentPosition());
        telemetry.update();


    }


}



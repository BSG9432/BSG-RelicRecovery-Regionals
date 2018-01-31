import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 299970 on 11/2/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DED")
@Disabled
public class TeleOp extends OpMode {

    public DcMotor rightM;
    public DcMotor leftM;
    public DcMotor rightC;
    public DcMotor leftC;
    public DcMotor liftT;
    public DcMotor liftB;
    public Servo rightH;
    public ColorSensor rightS;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        rightM = hardwareMap.dcMotor.get("rightM");
        leftM = hardwareMap.dcMotor.get("leftM");
        rightC = hardwareMap.dcMotor.get("rightC");
        leftC = hardwareMap.dcMotor.get("leftC");
        liftT = hardwareMap.dcMotor.get("liftT");
        liftB = hardwareMap.dcMotor.get("liftB");
        rightH = hardwareMap.servo.get("rightH");
        rightS = hardwareMap.colorSensor.get("rightS");

    }

    /*
 * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
 */
    @Override
    public void init_loop () {

        // NOTHING HERE

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start () {

        // NOTHING HERE

    }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */

    @Override
    public void loop () {

        if (Math.abs(gamepad1.right_stick_y) > .1) {
            rightM.setPower(-gamepad1.right_stick_y);
        }
        else {
            rightM.setPower(0);
        }
        if (Math.abs(gamepad1.left_stick_y) > .1) {
            leftM.setPower(gamepad1.left_stick_y);
        }
        else {
            leftM.setPower(0);
        }                                                           //DRIVE TRAIN

        if (gamepad1.left_bumper) {
            liftT.setPower(1);
            liftB.setPower(1);
        }
        else if (gamepad1.right_bumper) {
            liftT.setPower(-1);
            liftB.setPower(-1);
        }
        else {
            liftT.setPower(0);
            liftB.setPower(0);
        }                                                           //LIFT

        if (gamepad1.dpad_right) {
            rightH.setPosition(.9);
        }
        else {
            rightH.setPosition(.1);
        }                                                           //SERVO ARMS
        if (gamepad1.right_trigger > .1){
            rightC.setPower(.25);
            leftC.setPower(-.25);
        }
        else if (gamepad1.left_trigger > .1)
        {
            rightC.setPower(-.25);
            leftC.setPower(.25);
        }
        else {
            rightC.setPower(0);
            leftC.setPower(0);
        }                                                           //CLAMPS oof ;)gb

        telemetry.update();

    }

    /*
 * Code to run ONCE after the driver hits STOP
 */
    @Override
    public void stop() {

        // NOTHING HERE

    }

}



import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoTransition;

/**
 * Created by 299970 on 1/5/2018.
 */

@Autonomous(name="Blue 0 JEWEL", group="BLUE")
@Disabled
public class Blue0Jewel extends LinearOpMode {

    public DcMotor rightM;
    public DcMotor leftM;
    public DcMotor rightC;
    public DcMotor leftC;
    public DcMotor liftT;
    public DcMotor liftB;
    public Servo rightH;
    public ColorSensor rightS;

    static final int LED_CHANNEL = 5;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        rightM = hardwareMap.dcMotor.get("rightM");
        leftM = hardwareMap.dcMotor.get("leftM");
        rightC = hardwareMap.dcMotor.get("rightC");
        leftC = hardwareMap.dcMotor.get("leftC");
        liftT = hardwareMap.dcMotor.get("liftT");
        liftB = hardwareMap.dcMotor.get("liftB");
        rightH = hardwareMap.servo.get("rightH");
        rightS = hardwareMap.colorSensor.get("rightS");

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean bLedOn = true;

        telemetry.update();

        AutoTransition.transitionOnStop(this, "TeleOp");

        waitForStart();

        telemetry.update();

        sleep(1000);

        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(1000);

        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(500);

        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(1000);

        rightH.setPosition(1);
        rightH.setPosition(.9);
        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(2000);

        telemetry.update();

        telemetry.addData("RIGHT Clear", rightS.alpha());
        telemetry.addData("RIGHT Red  ", rightS.red());
        telemetry.addData("RIGHT Green", rightS.green());
        telemetry.addData("RIGHT Blue ", rightS.blue());
        telemetry.addData("RIGHT Hue", hsvValues[0]);

        telemetry.update();

        if (rightS.blue() < rightS.red()) {

            telemetry.update();

            rightM.setPower(-.25);
            rightC.setPower(-.1);
            leftC.setPower(.25);
            sleep(600);

            rightM.setPower(0);
            rightC.setPower(-.1);
            leftC.setPower(.25);
            sleep(500);

            rightH.setPosition(.1);
            rightC.setPower(-.1);
            leftC.setPower(.25);

            rightM.setPower(0);
            rightC.setPower(-.1);
            leftC.setPower(.25);
            sleep(500);

            rightM.setPower(.25);
            rightC.setPower(-.1);
            leftC.setPower(.25);
            sleep(600);

            rightM.setPower(0);
            rightC.setPower(-.1);
            leftC.setPower(.25);
            sleep(500);

        } else {

            telemetry.update();

            rightM.setPower(.25);
            rightC.setPower(-.1);
            leftC.setPower(.25);
            sleep(500);

            rightM.setPower(0);
            rightC.setPower(-.1);
            leftC.setPower(.25);
            sleep(500);

            rightH.setPosition(.1);
            rightC.setPower(-.1);
            leftC.setPower(.25);

            rightM.setPower(0);
            rightC.setPower(-.1);
            leftC.setPower(.25);
            sleep(500);

            rightM.setPower(-.5);
            rightC.setPower(-.1);
            leftC.setPower(.25);
            sleep(500);

            rightM.setPower(0);
            rightC.setPower(-.1);
            leftC.setPower(.25);
            sleep(500);
        }

        rightM.setPower(0);
        leftM.setPower(0);
        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(1000);        //pause

        rightM.setPower(.25);
        leftM.setPower(-.25);
        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(1000);        //go forward

        rightM.setPower(0);
        leftM.setPower(0);
        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(1000);        //pause

        rightM.setPower(-.5);
        leftM.setPower(-.5);
        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(1000);        //turn

        rightM.setPower(0);
        leftM.setPower(0);
        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(1000);        //pause

        rightM.setPower(.25);
        leftM.setPower(-.25);
        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(1000);        //go forward

        rightM.setPower(0);
        leftM.setPower(0);
        rightC.setPower(-.1);
        leftC.setPower(.25);
        sleep(1000);        //pause

        rightM.setPower(.5);
        leftM.setPower(.5);
        sleep(1000);        //turn to cryptobox

        rightM.setPower(0);
        leftM.setPower(0);
        sleep(1000);        //pause

        rightM.setPower(.25);
        leftM.setPower(-.25);
        sleep(1000);        //go forward

        rightM.setPower(0);
        leftM.setPower(0);
        sleep(1000);        //pause

        rightC.setPower(.1);
        leftC.setPower(-.1);
        sleep(1000);        //open clamp

        rightM.setPower(0);
        leftM.setPower(0);
        sleep(1000);        //pause

        rightM.setPower(-.25);
        leftM.setPower(.25);
        sleep(1000);        //back up

        rightC.setPower(-.1);
        leftC.setPower(.1);
        sleep(1000);        //close clamp

        rightM.setPower(0);
        leftM.setPower(0);
        sleep(1000);        //pause

        rightM.setPower(.25);
        leftM.setPower(-.25);
        sleep(1000);        //go forward

        rightM.setPower(-.25);
        leftM.setPower(.25);
        sleep(500);        //back up
    }
}

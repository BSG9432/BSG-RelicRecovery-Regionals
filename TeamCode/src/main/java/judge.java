
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoTransition;

@Autonomous(name="JUDGE", group="BLUE")
@Disabled
public class judge extends LinearOpMode {

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
    public void runOpMode() {

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

        AutoTransition.transitionOnStop(this, "NEW");

        waitForStart();

        sleep(1000);

        telemetry.update();

        rightH.setPosition(.8);
        sleep(1000);        //arm down

        telemetry.update();

        if (rightS.blue() < rightS.red()) {

            telemetry.update();

            rightM.setPower(-.25);
            sleep(500);     //back up

            rightM.setPower(0);
            sleep(500);     //pause

            rightH.setPosition(.1);    //arm up

            rightM.setPower(.25);
            sleep(500);     //forward

        } else {

            telemetry.update();

            rightM.setPower(.25);
            sleep(500);     //drive forward

            rightM.setPower(0);
            sleep(500);     //pause

            rightH.setPosition(.1);   //arm up

            rightM.setPower(-.25);
            sleep(500);     //back up

        }
    }
}
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 299970 on 1/20/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class NewTeleOp extends OpMode {

    public DcMotor rightM;
    public DcMotor leftM;
    public DcMotor rightC;
    public DcMotor leftC;
    public DcMotor liftT;
    public DcMotor liftB;
    public DcMotor rotate;
    public DcMotor relicLift;
    public Servo rightH;
    public ColorSensor rightS;

    @Override
    public void init() {

        rightM = hardwareMap.dcMotor.get("rightM");
        leftM = hardwareMap.dcMotor.get("leftM");
        rightC = hardwareMap.dcMotor.get("rightC");
        leftC = hardwareMap.dcMotor.get("leftC");
        liftT = hardwareMap.dcMotor.get("liftT");
        liftB = hardwareMap.dcMotor.get("liftB");
        rotate = hardwareMap.dcMotor.get("rotate");
        relicLift = hardwareMap.dcMotor.get("relicLift");
        rightH = hardwareMap.servo.get("rightH");
        rightS = hardwareMap.colorSensor.get("rightS");

    }

    @Override
    public void loop() {

        //int position = leftM.getCurrentPosition();
        //telemetry.addData("position", position);

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
            liftB.setPower(-1);
        }
        else if (gamepad1.right_bumper) {
            liftT.setPower(-1);
            liftB.setPower(1);
        }
        else {
            liftT.setPower(0);
            liftB.setPower(0);
        }                                                           //LIFT

        if (gamepad1.dpad_right) {
            rightH.setPosition(.1);
        }
        else {
            rightH.setPosition(.95);
        }                                                           //SERVO ARMS
        if (gamepad1.right_trigger > .1){
            rightC.setPower(.1);
            leftC.setPower(-.1);
        }
        else if (gamepad1.left_trigger > .1)
        {
            rightC.setPower(-.2);
            leftC.setPower(.2);
        }
        else {
            rightC.setPower(0);
            leftC.setPower(0);
        }                                                           //CLAMPS oof ;)gb

        if (gamepad2.left_bumper){
            rotate.setPower(-.25);
        }
        else if (gamepad2.right_bumper) {
            rotate.setPower(.25);
        }
        else {
            rotate.setPower(0);
        }

        if (gamepad2.y){
            relicLift.setPower(1);
        }
        else if (gamepad2.a){
            relicLift.setPower(-1);
        }
        else {
            relicLift.setPower(0);
        }
        telemetry.update();

    }
}

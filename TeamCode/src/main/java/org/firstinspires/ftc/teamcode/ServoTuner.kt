package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.round

@TeleOp(name = "Servo Tuner (FGC 2023)", group = "FGC 2023")
class ServoTuner: LinearOpMode() {
    private lateinit var filterLeft: Servo
    private lateinit var filterRight: Servo
    private lateinit var storageLeft: Servo
    private lateinit var storageRight: Servo

    private enum class SelectedServo {
        FILTER_LEFT,
        FILTER_RIGHT,
        STORAGE_LEFT,
        STORAGE_RIGHT
    }

    private var selectedServo = SelectedServo.FILTER_LEFT

    override fun runOpMode() {
        filterLeft = hardwareMap.get(Servo::class.java, "filterLeft")
        filterRight = hardwareMap.get(Servo::class.java, "filterRight")
        storageLeft = hardwareMap.get(Servo::class.java, "storageLeft")
        storageRight = hardwareMap.get(Servo::class.java, "storageRight")

        telemetry.addData("Servo tuner", "ready")
        telemetry.update()
        waitForStart()

        var last = System.currentTimeMillis()
        while (opModeIsActive()) {
            val delta = System.currentTimeMillis() - last
            last = System.currentTimeMillis()

            if(gamepad1.dpad_left) selectedServo = SelectedServo.FILTER_LEFT
            else if(gamepad1.dpad_right) selectedServo = SelectedServo.FILTER_RIGHT
            else if(gamepad1.dpad_up) selectedServo = SelectedServo.STORAGE_LEFT
            else if(gamepad1.dpad_down) selectedServo = SelectedServo.STORAGE_RIGHT

            val servo = when(selectedServo) {
                SelectedServo.FILTER_LEFT -> filterLeft
                SelectedServo.FILTER_RIGHT -> filterRight
                SelectedServo.STORAGE_LEFT -> storageLeft
                SelectedServo.STORAGE_RIGHT -> storageRight
            }

            if(gamepad1.b) break;

            servo.position = round(maxOf(minOf(servo.position - gamepad1.left_stick_y * delta/1000, 1.0), 0.0) * 100) / 100

            telemetry.addData("Selected servo", selectedServo);
            telemetry.addData("Position", servo.position);
            telemetry.update();
        }
    }
}
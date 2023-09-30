package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.roundToInt

@TeleOp(name = "Servo Tuner (FGC 2023)", group = "FGC 2023")
class ServoTuner: LinearOpMode() {
    private lateinit var filterLeft: Servo
    private lateinit var filterRight: Servo
    private lateinit var storageLeft: Servo
    private lateinit var storageRight: Servo
    private lateinit var clawLeft: Servo
    private lateinit var clawRight: Servo

    private val FILTER_DOWNWARDS = arrayOf(0.471, 0.454)
    private val FILTER_CENTER = arrayOf(0.0, 0.740)

    private val STORAGE_CLOSED = arrayOf(0.22, 1.0)
    private val STORAGE_OPEN = arrayOf(0.6, 0.62)

    private val HUGGGINATOR_CLOSED = arrayOf(0.2, 1.0)
    private val HUGGINATOR_OPEN = arrayOf(1.0, 0.0)

    private enum class SelectedServo {
        FILTER_LEFT,
        FILTER_RIGHT,
        STORAGE_LEFT,
        STORAGE_RIGHT,
        CLAW_LEFT,
        CLAW_RIGHT
    }

    private var selectedServo = SelectedServo.FILTER_LEFT

    override fun runOpMode() {
        filterLeft = hardwareMap.get(Servo::class.java, "filterLeft")
        filterRight = hardwareMap.get(Servo::class.java, "filterRight")
        storageLeft = hardwareMap.get(Servo::class.java, "storageLeft")
        storageRight = hardwareMap.get(Servo::class.java, "storageRight")
        clawLeft = hardwareMap.get(Servo::class.java, "clawLeft")
        clawRight = hardwareMap.get(Servo::class.java, "clawRight")

        telemetry.addData("Servo tuner", "ready")
        telemetry.update()
        waitForStart()

        var last = System.nanoTime()
        while (opModeIsActive()) {
            val delta = System.nanoTime() - last
            last = System.nanoTime()

            if(gamepad1.dpad_left) selectedServo = SelectedServo.FILTER_LEFT
            else if(gamepad1.dpad_right) selectedServo = SelectedServo.FILTER_RIGHT
            else if(gamepad1.dpad_up) selectedServo = SelectedServo.STORAGE_LEFT
            else if(gamepad1.dpad_down) selectedServo = SelectedServo.STORAGE_RIGHT
            else if(gamepad1.x) selectedServo = SelectedServo.CLAW_LEFT
            else if(gamepad1.b) selectedServo = SelectedServo.CLAW_RIGHT

            val servo = when(selectedServo) {
                SelectedServo.FILTER_LEFT -> filterLeft
                SelectedServo.FILTER_RIGHT -> filterRight
                SelectedServo.STORAGE_LEFT -> storageLeft
                SelectedServo.STORAGE_RIGHT -> storageRight
                SelectedServo.CLAW_LEFT -> clawLeft
                SelectedServo.CLAW_RIGHT -> clawRight
            }

            if((gamepad1.b && gamepad1.share) || (gamepad2.b && gamepad2.share)) break;

            if(gamepad2.dpad_up) {
                storageLeft.position = STORAGE_CLOSED[0]
                storageRight.position = STORAGE_CLOSED[1]
            } else if(gamepad2.dpad_down) {
                storageLeft.position = STORAGE_OPEN[0]
                storageRight.position = STORAGE_OPEN[1]
            }

            if(gamepad2.x) {
                filterLeft.position = FILTER_CENTER[0]
                filterRight.position = FILTER_CENTER[1]
            } else if(gamepad2.a) {
                filterLeft.position = FILTER_DOWNWARDS[0]
                filterRight.position = FILTER_DOWNWARDS[1]
            }

            if(gamepad2.left_bumper) {
                clawLeft.position = HUGGINATOR_OPEN[0]
                clawRight.position = HUGGINATOR_OPEN[1]
            } else if(gamepad2.right_bumper) {
                clawLeft.position = HUGGGINATOR_CLOSED[0]
                clawRight.position = HUGGGINATOR_CLOSED[1]
            }

            var storagePreset = ""
            if((storageLeft.position * 1000).roundToInt() == (STORAGE_CLOSED[0] * 1000).roundToInt() &&
                (storageRight.position * 1000).roundToInt() == (STORAGE_CLOSED[1] * 1000).roundToInt()) storagePreset = "CLOSED"
            if((storageLeft.position * 1000).roundToInt() == (STORAGE_OPEN[0] * 1000).roundToInt() &&
                (storageRight.position * 1000).roundToInt() == (STORAGE_OPEN[1] * 1000).roundToInt()) storagePreset = "OPEN"

            var filterPreset = ""
            if((filterLeft.position * 1000).roundToInt() == (FILTER_DOWNWARDS[0] * 1000).roundToInt() &&
                (filterRight.position * 1000).roundToInt() == (FILTER_DOWNWARDS[1] * 1000).roundToInt()) filterPreset = "DOWNWARDS"
            if((filterLeft.position * 1000).roundToInt() == (FILTER_CENTER[0] * 1000).roundToInt() &&
                (filterRight.position * 1000).roundToInt() == (FILTER_CENTER[1] * 1000).roundToInt()) filterPreset = "CENTER"

            var clawPreset = ""
            if((clawLeft.position * 1000).roundToInt() == (HUGGINATOR_OPEN[0] * 1000).roundToInt() &&
                (clawRight.position * 1000).roundToInt() == (HUGGINATOR_OPEN[1] * 1000).roundToInt()) clawPreset = "OPEN"
            if((clawLeft.position * 1000).roundToInt() == (HUGGGINATOR_CLOSED[0] * 1000).roundToInt() &&
                (clawRight.position * 1000).roundToInt() == (HUGGGINATOR_CLOSED[1] * 1000).roundToInt()) clawPreset = "DOWNWARDS"

            servo.position = maxOf(minOf(servo.position - gamepad1.left_stick_y * delta / 1e9, 1.0), 0.0)

            if(storagePreset.isNotEmpty()) telemetry.addData("STORAGE_PRESET", storagePreset)
            if(filterPreset.isNotEmpty()) telemetry.addData("FILTER_PRESET", filterPreset)
            if(clawPreset.isNotEmpty()) telemetry.addData("CLAW_PRESET", clawPreset)
            telemetry.addData("Selected servo", selectedServo)
            telemetry.addData("Position", "%.3f".format(servo.position))
            telemetry.update()
        }
    }
}
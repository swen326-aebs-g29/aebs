package aebs.simulator;

import aebs.simulator.faults.PerceptionFaultInjector;
import aebs.simulator.model.Vec2;
import aebs.simulator.perception.CameraReading;
import aebs.simulator.perception.RadarReading;
import aebs.simulator.perception.RedundantSensorFusion;
import aebs.simulator.perception.SensorHealthMonitor;
import aebs.simulator.perception.SimulatedSensors;
import aebs.simulator.perception.VehiclePerceptionSystem;
import aebs.simulator.perception.WheelSpeedReading;
import aebs.simulator.scenario.ScenarioEngine;
import aebs.simulator.ui.SimPanel;
import aebs.simulator.world.CarBlock;
import aebs.simulator.world.WorldState;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import java.awt.GraphicsEnvironment;
import java.util.Random;

public final class SimulatedWorldApp {
    public static void main(String[] args) {
        if (GraphicsEnvironment.isHeadless()) {
            startHeadless();
        } else {
            SwingUtilities.invokeLater(SimulatedWorldApp::startSwing);
        }
    }

    private static void startSwing() {
        double w = 520;
        double h = 720;
        double pxPerMeter = 10.0; // simple conversion for "meters"

        CarBlock ego = new CarBlock(
                "ego",
                CarBlock.Kind.EGO,
                new Vec2(w / 2.0 - 18, h - 140),
                new Vec2(0.0, -120.0),
                36,
                60
        );
        WorldState world = new WorldState(w, h, ego);

        ScenarioEngine scenario = new ScenarioEngine(
                /* seed */ 20260415L,
                /* laneXCenter */ w / 2.0,
                /* npcSpawnY */ -90
        );

        Random rng = new Random(20260415L);
        // Redundant sensor chains (A + B) for single-failure tolerance.
        SimulatedSensors sensorsA = new SimulatedSensors(pxPerMeter, /* wheelRadiusM */ 0.30, rng.nextLong());
        SimulatedSensors sensorsB = new SimulatedSensors(pxPerMeter, /* wheelRadiusM */ 0.30, rng.nextLong());
        PerceptionFaultInjector faultsA = new PerceptionFaultInjector(rng.nextLong());
        PerceptionFaultInjector faultsB = new PerceptionFaultInjector(rng.nextLong());

        // Control loop uses chain A for HUD/logic; perception output is fused A+B.
        SimPanel panel = new SimPanel(world, scenario, sensorsA, /* fps */ 60);

        JFrame frame = new JFrame("AEBS Simulator (animated blocks)");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setContentPane(panel);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
        panel.requestFocusInWindow();

        VehiclePerceptionSystem perception = new VehiclePerceptionSystem(
                new RadarReading[0],
                new CameraReading[0],
                new WheelSpeedReading[0]
        );
        SensorHealthMonitor health = new SensorHealthMonitor();

        // Update sensors at their specified frequencies:
        // - wheel speed: 10ms (100Hz)
        // - camera: 50ms (20Hz)
        // - radar: 100ms (10Hz)
        final int wheelPeriodMs = 10;
        final int cameraPeriodMs = 50;
        final int radarPeriodMs = 100;

        final long[] lastWheelMs = {0L};
        final long[] lastCameraMs = {0L};
        final long[] lastRadarMs = {0L};

        final RadarReading[][] radarA = {new RadarReading[0]};
        final RadarReading[][] radarB = {new RadarReading[0]};
        final CameraReading[][] cameraA = {new CameraReading[0]};
        final CameraReading[][] cameraB = {new CameraReading[0]};
        final WheelSpeedReading[][] wheelA = {new WheelSpeedReading[0]};
        final WheelSpeedReading[][] wheelB = {new WheelSpeedReading[0]};

        Timer sensorTimer = new Timer(wheelPeriodMs, e -> {
            double t = panel.simTimeS();

            long nowMs = System.currentTimeMillis();

            if (nowMs - lastWheelMs[0] >= wheelPeriodMs) {
                wheelA[0] = sensorsA.buildWheelSpeedReadings(panel.world());
                wheelA[0] = faultsA.applyWheel(t, wheelA[0]);
                wheelB[0] = sensorsB.buildWheelSpeedReadings(panel.world());
                wheelB[0] = faultsB.applyWheel(t, wheelB[0]);
                lastWheelMs[0] = nowMs;
            }

            if (nowMs - lastCameraMs[0] >= cameraPeriodMs) {
                cameraA[0] = sensorsA.buildCameraReadings(panel.world());
                cameraA[0] = faultsA.applyCamera(t, cameraA[0]);
                cameraB[0] = sensorsB.buildCameraReadings(panel.world());
                cameraB[0] = faultsB.applyCamera(t, cameraB[0]);
                lastCameraMs[0] = nowMs;
            }

            if (nowMs - lastRadarMs[0] >= radarPeriodMs) {
                radarA[0] = sensorsA.buildRadarReadings(panel.world());
                radarA[0] = faultsA.applyRadar(t, radarA[0]);
                radarB[0] = sensorsB.buildRadarReadings(panel.world());
                radarB[0] = faultsB.applyRadar(t, radarB[0]);
                lastRadarMs[0] = nowMs;
            }

            RadarReading[] radarF = RedundantSensorFusion.fuseRadar(radarA[0], radarB[0]);
            CameraReading[] camF = RedundantSensorFusion.fuseCamera(cameraA[0], cameraB[0]);
            WheelSpeedReading[] wheelF = RedundantSensorFusion.fuseWheel(wheelA[0], wheelB[0]);
            perception.updateAllSensors(radarF, camF, wheelF);

            SensorHealthMonitor.Health healthStatus = health.evaluate(t, radarF, camF, wheelF);
            System.out.println(formatPerception(perception, t, panel.world(), healthStatus));
        });
        sensorTimer.setCoalesce(true);
        sensorTimer.start();
    }

    private static void startHeadless() {
        double w = 520;
        double h = 720;
        double pxPerMeter = 10.0;

        CarBlock ego = new CarBlock(
                "ego",
                CarBlock.Kind.EGO,
                new Vec2(w / 2.0 - 18, h - 140),
                new Vec2(0.0, -120.0),
                36,
                60
        );
        WorldState world = new WorldState(w, h, ego);
        ScenarioEngine scenario = new ScenarioEngine(20260415L, w / 2.0, -90);

        Random rng = new Random(20260415L);
        SimulatedSensors sensorsA = new SimulatedSensors(pxPerMeter, 0.30, rng.nextLong());
        SimulatedSensors sensorsB = new SimulatedSensors(pxPerMeter, 0.30, rng.nextLong());
        PerceptionFaultInjector faultsA = new PerceptionFaultInjector(rng.nextLong());
        PerceptionFaultInjector faultsB = new PerceptionFaultInjector(rng.nextLong());

        VehiclePerceptionSystem perception = new VehiclePerceptionSystem(
                new RadarReading[0],
                new CameraReading[0],
                new WheelSpeedReading[0]
        );
        SensorHealthMonitor health = new SensorHealthMonitor();

        final double dt = 0.01; // 10ms base tick
        double t = 0.0;
        double wheelAcc = 0.0;
        double cameraAcc = 0.0;
        double radarAcc = 0.0;

        RadarReading[] radarA = new RadarReading[0];
        RadarReading[] radarB = new RadarReading[0];
        CameraReading[] cameraA = new CameraReading[0];
        CameraReading[] cameraB = new CameraReading[0];
        WheelSpeedReading[] wheelA = new WheelSpeedReading[0];
        WheelSpeedReading[] wheelB = new WheelSpeedReading[0];

        for (int i = 0; i < 2000; i++) { // 20s
            t += dt;
            world.setSimTimeS(t);
            scenario.step(world, dt);

            wheelAcc += dt;
            cameraAcc += dt;
            radarAcc += dt;

            if (wheelAcc >= 0.010) {
                wheelA = faultsA.applyWheel(t, sensorsA.buildWheelSpeedReadings(world));
                wheelB = faultsB.applyWheel(t, sensorsB.buildWheelSpeedReadings(world));
                wheelAcc = 0.0;
            }
            if (cameraAcc >= 0.050) {
                cameraA = faultsA.applyCamera(t, sensorsA.buildCameraReadings(world));
                cameraB = faultsB.applyCamera(t, sensorsB.buildCameraReadings(world));
                cameraAcc = 0.0;
            }
            if (radarAcc >= 0.100) {
                radarA = faultsA.applyRadar(t, sensorsA.buildRadarReadings(world));
                radarB = faultsB.applyRadar(t, sensorsB.buildRadarReadings(world));
                radarAcc = 0.0;
            }

            RadarReading[] radarF = RedundantSensorFusion.fuseRadar(radarA, radarB);
            CameraReading[] camF = RedundantSensorFusion.fuseCamera(cameraA, cameraB);
            WheelSpeedReading[] wheelF = RedundantSensorFusion.fuseWheel(wheelA, wheelB);
            perception.updateAllSensors(radarF, camF, wheelF);
            // Headless mode has no control loop; report brakeCmd as 0.
            world.setBrakeCommand(0.0);
            SensorHealthMonitor.Health healthStatus = health.evaluate(t, radarF, camF, wheelF);
            System.out.println(formatPerception(perception, t, world, healthStatus));
        }
    }

    private static String formatPerception(VehiclePerceptionSystem p, double tS, WorldState world, SensorHealthMonitor.Health h) {
        RadarReading[] r = p.getRadarReadings();
        CameraReading[] c = p.getCameraReadings();
        WheelSpeedReading[] w = p.getWheelSpeedReadings();

        String r0 = r.length == 0 ? "" : String.format("%.2fm@%s", r[0].distanceMetres(), r[0].speedObject());
        double rpm0 = w.length == 0 || w[0].rpm() == null ? 0.0 : w[0].rpm();

        return String.format("t=%.2f radarN=%d radar0=%s cameraN=%d wheelN=%d wheel0_rpm=%.1f brakeCmd=%.2f brakeAlert=%s sensorHealth=%s",
                tS, r.length, r0, c.length, w.length, rpm0, world.brakeCommand(), world.driverBrakeAlert() ? "YES" : "NO",
                (h == null || h.ok()) ? "OK" : h.summary());
    }
}

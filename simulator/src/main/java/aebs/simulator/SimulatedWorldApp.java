package aebs.simulator;

import aebs.simulator.faults.PerceptionFaultInjector;
import aebs.simulator.model.Vec2;
import aebs.simulator.perception.CameraReading;
import aebs.simulator.perception.RadarReading;
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
        SimulatedSensors sensors = new SimulatedSensors(pxPerMeter, /* wheelRadiusM */ 0.30, rng.nextLong());
        PerceptionFaultInjector faults = new PerceptionFaultInjector(rng.nextLong());

        SimPanel panel = new SimPanel(world, scenario, sensors, /* fps */ 60);

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

        final RadarReading[][] radar = {new RadarReading[0]};
        final CameraReading[][] camera = {new CameraReading[0]};
        final WheelSpeedReading[][] wheel = {new WheelSpeedReading[0]};

        Timer sensorTimer = new Timer(wheelPeriodMs, e -> {
            double t = panel.simTimeS();

            long nowMs = System.currentTimeMillis();

            if (nowMs - lastWheelMs[0] >= wheelPeriodMs) {
                wheel[0] = sensors.buildWheelSpeedReadings(panel.world());
                wheel[0] = faults.applyWheel(t, wheel[0]);
                lastWheelMs[0] = nowMs;
            }

            if (nowMs - lastCameraMs[0] >= cameraPeriodMs) {
                camera[0] = sensors.buildCameraReadings(panel.world());
                camera[0] = faults.applyCamera(t, camera[0]);
                lastCameraMs[0] = nowMs;
            }

            if (nowMs - lastRadarMs[0] >= radarPeriodMs) {
                radar[0] = sensors.buildRadarReadings(panel.world());
                radar[0] = faults.applyRadar(t, radar[0]);
                lastRadarMs[0] = nowMs;
            }

            perception.updateAllSensors(radar[0], camera[0], wheel[0]);

            System.out.println(formatPerception(perception, t));
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
        SimulatedSensors sensors = new SimulatedSensors(pxPerMeter, 0.30, rng.nextLong());
        PerceptionFaultInjector faults = new PerceptionFaultInjector(rng.nextLong());

        VehiclePerceptionSystem perception = new VehiclePerceptionSystem(
                new RadarReading[0],
                new CameraReading[0],
                new WheelSpeedReading[0]
        );

        final double dt = 0.01; // 10ms base tick
        double t = 0.0;
        double wheelAcc = 0.0;
        double cameraAcc = 0.0;
        double radarAcc = 0.0;

        RadarReading[] radar = new RadarReading[0];
        CameraReading[] camera = new CameraReading[0];
        WheelSpeedReading[] wheel = new WheelSpeedReading[0];

        for (int i = 0; i < 2000; i++) { // 20s
            t += dt;
            world.setSimTimeS(t);
            scenario.step(world, dt);

            wheelAcc += dt;
            cameraAcc += dt;
            radarAcc += dt;

            if (wheelAcc >= 0.010) {
                wheel = faults.applyWheel(t, sensors.buildWheelSpeedReadings(world));
                wheelAcc = 0.0;
            }
            if (cameraAcc >= 0.050) {
                camera = faults.applyCamera(t, sensors.buildCameraReadings(world));
                cameraAcc = 0.0;
            }
            if (radarAcc >= 0.100) {
                radar = faults.applyRadar(t, sensors.buildRadarReadings(world));
                radarAcc = 0.0;
            }

            perception.updateAllSensors(radar, camera, wheel);
            System.out.println(formatPerception(perception, t));
        }
    }

    private static String formatPerception(VehiclePerceptionSystem p, double tS) {
        RadarReading[] r = p.getRadarReadings();
        CameraReading[] c = p.getCameraReadings();
        WheelSpeedReading[] w = p.getWheelSpeedReadings();

        String r0 = r.length == 0 ? "" : String.format("%.2fm@%s", r[0].distanceMetres(), r[0].speedObject());
        double rpm0 = w.length == 0 || w[0].rpm() == null ? 0.0 : w[0].rpm();

        return String.format("t=%.2f radarN=%d radar0=%s cameraN=%d wheelN=%d wheel0_rpm=%.1f",
                tS, r.length, r0, c.length, w.length, rpm0);
    }
}

import 'dart:async';
import 'dart:math';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'package:activity_recognition_flutter/activity_recognition_flutter.dart';
import 'package:pedometer/pedometer.dart';
import 'package:permission_handler/permission_handler.dart';

void main() {
  runApp(const MallSensorDemoApp());
}

class MallSensorDemoApp extends StatelessWidget {
  const MallSensorDemoApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Mall Indoor Sensor + Motion Demo',
      debugShowCheckedModeBanner: false,
      theme: ThemeData.dark(),
      home: const SensorNavigationPage(),
    );
  }
}

class SensorNavigationPage extends StatefulWidget {
  const SensorNavigationPage({super.key});

  @override
  State<SensorNavigationPage> createState() => _SensorNavigationPageState();
}

class _SensorNavigationPageState extends State<SensorNavigationPage> {
  // ---- Oldingi sensorlar ----
  StreamSubscription<AccelerometerEvent>? _accelSub;
  StreamSubscription<GyroscopeEvent>? _gyroSub;
  StreamSubscription<MagnetometerEvent>? _magSub;

  Offset _position = const Offset(0, 0);
  Offset _velocity = Offset.zero;
  double _headingRad = 0;

  AccelerometerEvent? _lastAccel;
  GyroscopeEvent? _lastGyro;
  MagnetometerEvent? _lastMag;

  static const double _dt = 0.05; // ~20Hz demo

  // ---- Motion & Fitness: Activity + Steps ----
  final ActivityRecognition _activityRecognition = ActivityRecognition();
  StreamSubscription<ActivityEvent>? _activitySub;
  ActivityType? _currentActivity;
  double? _activityConfidence;

  StreamSubscription<StepCount>? _stepSub;
  int _steps = 0;

  @override
  void initState() {
    super.initState();
    _initSensors();
    _initMotionAndFitness();
  }

  void _initSensors() {
    // Accelerometer
    _accelSub = accelerometerEventStream().listen((event) {
      _lastAccel = event;

      const double threshold = 1.2;
      final ax = event.x;
      final ay = event.y;

      double effectiveAx = 0;
      double effectiveAy = 0;

      if (ax.abs() > threshold) {
        effectiveAx = ax.sign * (ax.abs() - threshold);
      }
      if (ay.abs() > threshold) {
        effectiveAy = ay.sign * (ay.abs() - threshold);
      }

      _velocity += Offset(effectiveAx, -effectiveAy) * _dt;
      _position += _velocity * _dt * 20;

      setState(() {});
    });

    // Gyroscope -> heading
    _gyroSub = gyroscopeEventStream().listen((event) {
      _lastGyro = event;

      _headingRad += event.z * _dt;
      _headingRad = atan2(sin(_headingRad), cos(_headingRad));

      setState(() {});
    });

    // Magnetometer – hozircha UI uchun
    _magSub = magnetometerEventStream().listen((event) {
      _lastMag = event;
      setState(() {});
    });
  }

  Future<void> _initMotionAndFitness() async {
    // 1. Permission so‘rash (asosan Android uchun)
    if (Platform.isAndroid) {
      final status = await Permission.activityRecognition.request();
      if (!status.isGranted) {
        debugPrint('Activity Recognition permission berilmadi');
        return;
      }
    }

    // 2. Activity stream
    _activitySub = _activityRecognition.activityStream().listen(
      (ActivityEvent event) {
        setState(() {
          _currentActivity = event.type;
          _activityConfidence = event.confidence.toDouble();
        });
      },
      onError: (e) {
        debugPrint('Activity error: $e');
      },
    );

    // 3. Step count stream
    try {
      _stepSub = Pedometer.stepCountStream.listen(
        (StepCount event) {
          setState(() {
            _steps = event.steps;
          });
        },
        onError: (e) {
          debugPrint('StepCount error: $e');
        },
      );
    } catch (e) {
      debugPrint('Step stream init error: $e');
    }
  }

  @override
  void dispose() {
    _accelSub?.cancel();
    _gyroSub?.cancel();
    _magSub?.cancel();

    _activitySub?.cancel();
    _stepSub?.cancel();

    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Indoor Sensor + Motion Demo'),
        actions: [IconButton(onPressed: _resetPosition, icon: const Icon(Icons.refresh), tooltip: 'Reset pozitsiya')],
      ),
      body: LayoutBuilder(
        builder: (context, constraints) {
          final center = Offset(constraints.maxWidth / 2, constraints.maxHeight / 2);

          final clamped = Offset(center.dx + _position.dx, center.dy + _position.dy);

          final dotPos = Offset(
            clamped.dx.clamp(20, constraints.maxWidth - 20),
            clamped.dy.clamp(20, constraints.maxHeight - 20),
          );

          return Column(
            children: [
              Expanded(
                flex: 3,
                child: Stack(
                  children: [
                    Positioned.fill(child: CustomPaint(painter: _GridPainter())),
                    Center(
                      child: Container(
                        width: 10,
                        height: 10,
                        decoration: BoxDecoration(color: Colors.grey.withOpacity(0.5), shape: BoxShape.circle),
                      ),
                    ),
                    Center(
                      child: Transform.rotate(
                        angle: _headingRad,
                        child: Icon(Icons.navigation, size: 40, color: Colors.blueAccent.withOpacity(0.9)),
                      ),
                    ),
                    Positioned(
                      left: dotPos.dx - 10,
                      top: dotPos.dy - 10,
                      child: Container(
                        width: 20,
                        height: 20,
                        decoration: BoxDecoration(
                          color: Colors.orangeAccent,
                          shape: BoxShape.circle,
                          boxShadow: [BoxShadow(color: Colors.black.withOpacity(0.4), blurRadius: 4)],
                        ),
                      ),
                    ),
                  ],
                ),
              ),
              const Divider(height: 1),
              Expanded(flex: 2, child: _buildSensorInfo()),
            ],
          );
        },
      ),
    );
  }

  Widget _buildSensorInfo() {
    String accelText = _lastAccel != null
        ? 'x: ${_lastAccel!.x.toStringAsFixed(2)}, '
              'y: ${_lastAccel!.y.toStringAsFixed(2)}, '
              'z: ${_lastAccel!.z.toStringAsFixed(2)}'
        : 'N/A';

    String gyroText = _lastGyro != null
        ? 'x: ${_lastGyro!.x.toStringAsFixed(2)}, '
              'y: ${_lastGyro!.y.toStringAsFixed(2)}, '
              'z: ${_lastGyro!.z.toStringAsFixed(2)}'
        : 'N/A';

    String magText = _lastMag != null
        ? 'x: ${_lastMag!.x.toStringAsFixed(2)}, '
              'y: ${_lastMag!.y.toStringAsFixed(2)}, '
              'z: ${_lastMag!.z.toStringAsFixed(2)}'
        : 'N/A';

    final activityName = _currentActivity?.name ?? 'unknown';
    final confStr = _activityConfidence != null ? '${_activityConfidence!.toStringAsFixed(0)}%' : 'N/A';

    return Padding(
      padding: const EdgeInsets.all(16),
      child: DefaultTextStyle(
        style: const TextStyle(fontSize: 14),
        child: SingleChildScrollView(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text('Sensor + Motion ma’lumotlari:', style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16)),
              const SizedBox(height: 8),
              Text('Heading (rad): ${_headingRad.toStringAsFixed(2)}'),
              const SizedBox(height: 4),
              Text('Accelerometer: $accelText'),
              const SizedBox(height: 4),
              Text('Gyroscope: $gyroText'),
              const SizedBox(height: 4),
              Text('Magnetometer: $magText'),
              const SizedBox(height: 12),
              Text('Activity (motion): $activityName (confidence: $confStr)'),
              Text('Steps (qadamlar): $_steps'),
              const SizedBox(height: 12),
              Text(
                'Pozitsiya (relativ): '
                'x=${_position.dx.toStringAsFixed(1)}, '
                'y=${_position.dy.toStringAsFixed(1)}',
              ),
              Text(
                'Tezlik (relativ): '
                'vx=${_velocity.dx.toStringAsFixed(2)}, '
                'vy=${_velocity.dy.toStringAsFixed(2)}',
              ),
              const SizedBox(height: 12),
              const Text(
                '⚠️ Bu hamon demo:\n'
                '- Activity & Steps – tizimdan keladigan ma’lumotlar\n'
                '- PDR (step-based navigatsiya) qo‘shsang, qadam + heading orqali \n'
                '  foydalanuvchini mall xaritasida siljitish mumkin bo‘ladi.',
              ),
            ],
          ),
        ),
      ),
    );
  }

  void _resetPosition() {
    setState(() {
      _position = Offset.zero;
      _velocity = Offset.zero;
      _headingRad = 0;
      _steps = 0;
    });
  }
}

class _GridPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..strokeWidth = 0.5
      ..color = Colors.white.withOpacity(0.1);

    const double step = 20;

    for (double x = 0; x < size.width; x += step) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }
    for (double y = 0; y < size.height; y += step) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}
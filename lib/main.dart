import 'dart:async';
import 'dart:math';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'package:activity_recognition_flutter/activity_recognition_flutter.dart';
import 'package:geolocator/geolocator.dart' hide ActivityType;
import 'package:pedometer/pedometer.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:ar_flutter_plugin/ar_flutter_plugin.dart';
import 'package:ar_flutter_plugin/datatypes/config_planedetection.dart';
import 'package:ar_flutter_plugin/managers/ar_anchor_manager.dart';
import 'package:ar_flutter_plugin/managers/ar_location_manager.dart';
import 'package:ar_flutter_plugin/managers/ar_object_manager.dart';
import 'package:ar_flutter_plugin/managers/ar_session_manager.dart';

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
  StreamSubscription<BarometerEvent>? _barometerSub;
  StreamSubscription<Position>? _positionSub;

  Offset _position = Offset.zero;
  Offset _inertialPosition = Offset.zero;
  Offset _stepPosition = Offset.zero;
  Offset _velocity = Offset.zero;
  Offset? _lastFusedMeters;
  double _odometerMeters = 0;
  double _headingRad = 0;
  final List<Offset> _trail = <Offset>[];
  int _trailVersion = 0;
  int? _stepBaseline;
  int _relativeSteps = 0;
  double _motionReliability = 0.4;
  double _lastAccelMagnitude = 0;
  final _emaX = _ExponentialMovingAverage(alpha: 0.25);
  final _emaY = _ExponentialMovingAverage(alpha: 0.25);
  double _gravityX = 0;
  double _gravityY = 0;
  double _gravityZ = 0;
  DateTime? _lastAccelSample;
  DateTime? _lastGyroSample;
  DateTime? _lastGpsFixTime;

  AccelerometerEvent? _lastAccel;
  GyroscopeEvent? _lastGyro;
  MagnetometerEvent? _lastMag;
  Position? _lastPosition;

  Offset? _gpsOffset;
  Offset? _gpsFilteredOffset;
  double _gpsAccuracyMeters = double.infinity;
  double _gpsSpeedMps = 0;
  double? _gpsAltitude;
  double? _gpsHeadingDeg;
  double _gpsTrust = 0;
  String _geoStatusMessage = 'GPS turibdi...';
  double? _pressureHPa;
  double? _barometricAltitude;

  final _MallGeoProjector _geoProjector = _MallGeoProjector();
  final _MallGraph _mallGraph = _MallGraph.preset();
  late final List<int> _mallFloors = _mallGraph.floors;
  int _selectedFloor = _MallGraph.defaultFloor;
  String _targetNodeId = _MallGraph.defaultDestinationId;
  List<_NavInstruction> _navInstructions = <_NavInstruction>[];
  _MallNode? _currentNode;
  double? _relativeInstructionBearing;
  double? _distanceToNextInstruction;
  _NavInstruction? _activeInstruction;

  bool _arModeEnabled = false;
  String _arStatus = 'AR idle';
  ARSessionManager? _arSessionManager;

  static const double _dt = 0.05; // fallback sampling time
  static const double _metersToPixels = 55;
  static const double _seaLevelPressureHPa = 1013.25;

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
    _initGeolocation();
    if (_mallFloors.isNotEmpty && !_mallFloors.contains(_selectedFloor)) {
      _selectedFloor = _mallFloors.first;
    }
    _ensureTargetMatchesFloor();
  }

  void _initSensors() {
    // Accelerometer
    _accelSub = accelerometerEventStream().listen((event) {
      _lastAccel = event;

      final now = DateTime.now();
      final double dtSeconds;
      if (_lastAccelSample == null) {
        dtSeconds = _dt;
      } else {
        final elapsed = now.difference(_lastAccelSample!).inMicroseconds / 1e6;
        dtSeconds = elapsed.clamp(0.01, 0.2);
      }
      _lastAccelSample = now;

      const double gravityAlpha = 0.92;
      _gravityX = gravityAlpha * _gravityX + (1 - gravityAlpha) * event.x;
      _gravityY = gravityAlpha * _gravityY + (1 - gravityAlpha) * event.y;
      _gravityZ = gravityAlpha * _gravityZ + (1 - gravityAlpha) * event.z;

      final double linearX = event.x - _gravityX;
      final double linearY = event.y - _gravityY;

      final double smoothedX = _emaX.next(linearX);
      final double smoothedY = _emaY.next(linearY);

      final Offset acceleration = Offset(smoothedX, -smoothedY);
      final Offset filteredAccel = _applyDeadZone(acceleration, 0.015);
      final Offset scaledAccel = Offset(filteredAccel.dx * _metersToPixels, filteredAccel.dy * _metersToPixels);

      _velocity = Offset(
        (_velocity.dx + scaledAccel.dx * dtSeconds) * 0.995,
        (_velocity.dy + scaledAccel.dy * dtSeconds) * 0.995,
      );
      _inertialPosition = _limitOffset(_inertialPosition + _velocity * dtSeconds);

      _lastAccelMagnitude = acceleration.distance;
      _motionReliability = _computeReliability(filteredAccel);

      _updateFusedPosition();

      setState(() {});
    });

    // Gyroscope -> heading
    _gyroSub = gyroscopeEventStream().listen((event) {
      _lastGyro = event;

      final now = DateTime.now();
      final double dtSeconds;
      if (_lastGyroSample == null) {
        dtSeconds = _dt;
      } else {
        final elapsed = now.difference(_lastGyroSample!).inMicroseconds / 1e6;
        dtSeconds = elapsed.clamp(0.005, 0.2);
      }
      _lastGyroSample = now;

      _headingRad = _normalizeAngle(_headingRad + event.z * dtSeconds);

      setState(() {});
    });

    // Magnetometer – hozircha UI uchun
    _magSub = magnetometerEventStream().listen((event) {
      _lastMag = event;
      final double magHeading = atan2(event.y, event.x);
      _headingRad = _blendAngles(_headingRad, magHeading, 0.02);
      setState(() {});
    });

    _barometerSub = barometerEventStream().listen(
      (event) {
        final double pressure = event.pressure; // hPa
        _pressureHPa = pressure;
        _barometricAltitude = _pressureToAltitudeMeters(pressure);
        setState(() {});
      },
      onError: (e) {
        debugPrint('Barometer error: $e');
      },
    );
  }

  Future<void> _initMotionAndFitness() async {
    // 1. Kamera ruxsatini har ikkala platformada so‘rash
    final PermissionStatus cameraStatus = await Permission.camera.request();
    if (!cameraStatus.isGranted) {
      debugPrint('Camera permission berilmadi: $cameraStatus');
    }

    // 2. Activity recognition faqat Android uchun
    if (Platform.isAndroid) {
      final PermissionStatus activityStatus = await Permission.activityRecognition.request();
      if (!activityStatus.isGranted) {
        debugPrint('Activity Recognition permission berilmadi');
        return;
      }
    }

    // 3. Activity stream
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

    // 4. Step count stream
    try {
      _stepSub = Pedometer.stepCountStream.listen(
        (StepCount event) {
          _handleStepEvent(event.steps);
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

  Future<void> _initGeolocation() async {
    final bool serviceEnabled = await Geolocator.isLocationServiceEnabled();
    if (!serviceEnabled) {
      setState(() {
        _geoStatusMessage = 'Location services are disabled';
      });
      return;
    }

    LocationPermission permission = await Geolocator.checkPermission();
    if (permission == LocationPermission.denied) {
      permission = await Geolocator.requestPermission();
    }
    if (permission == LocationPermission.deniedForever || permission == LocationPermission.denied) {
      setState(() {
        _geoStatusMessage = 'Location permission denied';
      });
      return;
    }

    setState(() {
      _geoStatusMessage = 'Acquiring GPS lock...';
    });

    const LocationSettings settings = LocationSettings(accuracy: LocationAccuracy.bestForNavigation, distanceFilter: 0);

    _positionSub = Geolocator.getPositionStream(locationSettings: settings).listen(
      (position) {
        _handlePositionSample(position);
      },
      onError: (e) {
        debugPrint('Position error: $e');
        setState(() {
          _geoStatusMessage = 'GPS error: $e';
        });
      },
    );

    final Position? cached = await Geolocator.getLastKnownPosition();
    if (cached != null) {
      _handlePositionSample(cached);
    }
  }

  @override
  void dispose() {
    _accelSub?.cancel();
    _gyroSub?.cancel();
    _magSub?.cancel();
    _barometerSub?.cancel();

    _activitySub?.cancel();
    _stepSub?.cancel();
    _positionSub?.cancel();
    _disposeArSession();

    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Indoor Sensor + Motion Demo'),
        actions: [
          IconButton(
            onPressed: _isArSupported ? () => _toggleArMode(context) : null,
            icon: Icon(_arModeEnabled ? Icons.view_in_ar : Icons.view_in_ar_outlined),
            tooltip: 'AR navigator',
          ),
          IconButton(onPressed: _resetPosition, icon: const Icon(Icons.refresh), tooltip: 'Reset pozitsiya'),
        ],
      ),
      body: Column(
        children: [
          if (_arModeEnabled) _buildArPanel(),
          Expanded(
            child: Column(
              children: [
                Expanded(
                  flex: 3,
                  child: LayoutBuilder(
                    builder: (context, canvasConstraints) {
                      final center = Offset(canvasConstraints.maxWidth / 2, canvasConstraints.maxHeight / 2);
                      final constrained = Offset(center.dx + _position.dx, center.dy + _position.dy);
                      final dotPos = Offset(
                        constrained.dx.clamp(20, canvasConstraints.maxWidth - 20),
                        constrained.dy.clamp(20, canvasConstraints.maxHeight - 20),
                      );

                      final double reliability = _motionReliability.clamp(0.0, 1.0);
                      final double motionDotSize = _lerpDouble(16, 32, reliability);
                      final Color dotColor =
                          Color.lerp(Colors.deepOrangeAccent, Colors.amberAccent, reliability) ?? Colors.orangeAccent;
                      final Color glowColor = dotColor.withValues(alpha: (0.4 + reliability * 0.2).clamp(0.0, 1.0));
                      final List<Offset> trailPoints = List<Offset>.from(_trail);
                      final String reliabilityLabel = '${(reliability * 100).toStringAsFixed(0)}%';

                      return Stack(
                        children: [
                          Positioned.fill(
                            child: DecoratedBox(
                              decoration: BoxDecoration(
                                gradient: LinearGradient(
                                  colors: [Colors.blueGrey.shade900, Colors.blueGrey.shade800, Colors.black],
                                  begin: Alignment.topLeft,
                                  end: Alignment.bottomRight,
                                ),
                              ),
                            ),
                          ),
                          Positioned.fill(child: CustomPaint(painter: _GridPainter())),
                          Positioned.fill(
                            child: CustomPaint(
                              painter: _TrailPainter(points: trailPoints, version: _trailVersion),
                            ),
                          ),
                          Center(
                            child: Container(
                              width: 14,
                              height: 14,
                              decoration: BoxDecoration(
                                color: Colors.white.withValues(alpha: 0.3),
                                shape: BoxShape.circle,
                              ),
                            ),
                          ),
                          Center(
                            child: Transform.rotate(
                              angle: _headingRad,
                              child: Icon(
                                Icons.navigation,
                                size: 44,
                                color: Colors.lightBlueAccent.withValues(alpha: 0.9),
                              ),
                            ),
                          ),
                          Positioned(
                            left: dotPos.dx - motionDotSize / 2,
                            top: dotPos.dy - motionDotSize / 2,
                            child: AnimatedContainer(
                              duration: const Duration(milliseconds: 250),
                              width: motionDotSize,
                              height: motionDotSize,
                              decoration: BoxDecoration(
                                color: dotColor,
                                shape: BoxShape.circle,
                                boxShadow: [BoxShadow(color: glowColor, blurRadius: 25, spreadRadius: 4)],
                              ),
                            ),
                          ),
                          Positioned(
                            top: 16,
                            right: 16,
                            child: Container(
                              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                              decoration: BoxDecoration(
                                color: Colors.black.withValues(alpha: 0.35),
                                borderRadius: BorderRadius.circular(12),
                                border: Border.all(color: Colors.white12),
                              ),
                              child: Column(
                                crossAxisAlignment: CrossAxisAlignment.end,
                                children: [
                                  const Text('Motion accuracy', style: TextStyle(fontSize: 12, color: Colors.white70)),
                                  Text(
                                    reliabilityLabel,
                                    style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
                                  ),
                                ],
                              ),
                            ),
                          ),
                        ],
                      );
                    },
                  ),
                ),
                const Divider(height: 1),
                Expanded(flex: 2, child: _buildSensorInfo()),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildArPanel() {
    if (!_isArSupported) {
      return Padding(
        padding: const EdgeInsets.all(16),
        child: Container(
          padding: const EdgeInsets.all(16),
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(20),
            color: Colors.white.withValues(alpha: 0.05),
            border: Border.all(color: Colors.white12),
          ),
          child: const Text('AR navigatsiya faqat mobil qurilmalarda ishlaydi.'),
        ),
      );
    }
    final double arrowAngle = _relativeInstructionBearing ?? 0;
    final String nextText = _activeInstruction?.text ?? 'Ko‘rsatma kalibrlanmoqda...';
    final String distanceLabel = _distanceToNextInstruction != null
        ? '${_distanceToNextInstruction!.toStringAsFixed(0)} m'
        : '--';

    return Container(
      margin: const EdgeInsets.fromLTRB(16, 16, 16, 8),
      height: 260,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(24),
        border: Border.all(color: Colors.white12),
        boxShadow: [BoxShadow(color: Colors.black.withValues(alpha: 0.35), blurRadius: 20)],
      ),
      child: ClipRRect(
        borderRadius: BorderRadius.circular(24),
        child: Stack(
          children: [
            ARView(onARViewCreated: _onARViewCreated, planeDetectionConfig: PlaneDetectionConfig.horizontalAndVertical),
            Positioned.fill(
              child: IgnorePointer(
                child: Column(
                  children: [
                    Align(
                      alignment: Alignment.topRight,
                      child: Container(
                        margin: const EdgeInsets.all(12),
                        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                        decoration: BoxDecoration(
                          color: Colors.black.withValues(alpha: 0.5),
                          borderRadius: BorderRadius.circular(20),
                        ),
                        child: Text(_arStatus, style: const TextStyle(fontSize: 12)),
                      ),
                    ),
                    Expanded(
                      child: Center(
                        child: Transform.rotate(
                          angle: arrowAngle,
                          child: Icon(Icons.navigation, size: 90, color: Colors.amberAccent.withValues(alpha: 0.95)),
                        ),
                      ),
                    ),
                    Container(
                      width: double.infinity,
                      padding: const EdgeInsets.all(16),
                      decoration: BoxDecoration(
                        color: Colors.black.withValues(alpha: 0.55),
                        borderRadius: const BorderRadius.only(
                          topLeft: Radius.circular(24),
                          topRight: Radius.circular(24),
                        ),
                      ),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Text(nextText, style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 16)),
                          const SizedBox(height: 4),
                          Text('Keyingi nuqta: $distanceLabel'),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  void _toggleArMode(BuildContext context) {
    if (!_isArSupported) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('AR faqat ARKit/ARCore mos qurilmalarda mavjud')));
      return;
    }
    setState(() {
      _arModeEnabled = !_arModeEnabled;
      if (_arModeEnabled) {
        _arStatus = 'AR ishga tushirilmoqda...';
      } else {
        _disposeArSession();
      }
    });
  }

  void _onARViewCreated(
    ARSessionManager arSessionManager,
    ARObjectManager arObjectManager,
    ARAnchorManager _,
    ARLocationManager __,
  ) {
    _arSessionManager = arSessionManager;

    _arSessionManager?.onInitialize(
      showPlanes: true,
      showFeaturePoints: false,
      showWorldOrigin: false,
      handleTaps: false,
    );
    arObjectManager.onInitialize();

    setState(() {
      _arStatus = 'AR tayyor';
    });
  }

  void _disposeArSession() {
    _arSessionManager?.dispose();
    _arSessionManager = null;
    _arStatus = 'AR idle';
  }

  void _ensureTargetMatchesFloor() {
    final _MallNode? targetNode = _mallGraph.findNode(_targetNodeId);
    if (targetNode != null && targetNode.floor == _selectedFloor) {
      return;
    }
    final _MallNode? fallback = _mallGraph.firstNodeOnFloor(_selectedFloor);
    if (fallback != null) {
      _targetNodeId = fallback.id;
    }
  }

  void _updateNavigationSolution() {
    _ensureTargetMatchesFloor();
    final _MallNode? nearest = _mallGraph.nearestNode(_position, floor: _selectedFloor);
    _currentNode = nearest;
    if (nearest == null) {
      _navInstructions = <_NavInstruction>[];
      _activeInstruction = null;
      _relativeInstructionBearing = null;
      _distanceToNextInstruction = null;
      return;
    }
    final List<String> path = _mallGraph.shortestPath(nearest.id, _targetNodeId, floor: _selectedFloor);
    _navInstructions = _mallGraph.describePath(
      path: path,
      metersToPixels: _metersToPixels,
      currentHeading: _headingRad,
    );
    if (_navInstructions.isNotEmpty) {
      _activeInstruction = _navInstructions.first;
      _relativeInstructionBearing = _navInstructions.first.relativeHeading;
      _distanceToNextInstruction = _navInstructions.first.distanceMeters;
    } else {
      _activeInstruction = null;
      _relativeInstructionBearing = null;
      _distanceToNextInstruction = null;
    }
  }

  Widget _buildSensorInfo() {
    final accelText = _lastAccel != null
        ? 'x: ${_lastAccel!.x.toStringAsFixed(2)}, '
              'y: ${_lastAccel!.y.toStringAsFixed(2)}, '
              'z: ${_lastAccel!.z.toStringAsFixed(2)}'
        : 'N/A';

    final gyroText = _lastGyro != null
        ? 'x: ${_lastGyro!.x.toStringAsFixed(2)}, '
              'y: ${_lastGyro!.y.toStringAsFixed(2)}, '
              'z: ${_lastGyro!.z.toStringAsFixed(2)}'
        : 'N/A';

    final magText = _lastMag != null
        ? 'x: ${_lastMag!.x.toStringAsFixed(2)}, '
              'y: ${_lastMag!.y.toStringAsFixed(2)}, '
              'z: ${_lastMag!.z.toStringAsFixed(2)}'
        : 'N/A';

    final activityName = _formatActivityName(_currentActivity);
    final confStr = _activityConfidence != null ? '${_activityConfidence!.toStringAsFixed(0)}%' : 'N/A';
    final String headingDeg = (_headingRad * 180 / pi).toStringAsFixed(0);
    final String reliabilityPercent = (_motionReliability.clamp(0.0, 1.0) * 100).toStringAsFixed(0);
    final String velocityMeters = (_velocity.distance / _metersToPixels).abs().toStringAsFixed(2);
    final String accelMagnitude = _lastAccelMagnitude.toStringAsFixed(2);
    final String stepLength = _dynamicStepLengthMeters().toStringAsFixed(2);
    final Color reliabilityColor =
        Color.lerp(Colors.redAccent, Colors.lightGreenAccent, _motionReliability.clamp(0.0, 1.0)) ??
        Colors.lightGreenAccent;
    final String gpsAccuracyLabel = _gpsAccuracyMeters.isFinite ? '${_gpsAccuracyMeters.toStringAsFixed(1)} m' : 'N/A';
    final String altitudeLabel = _gpsAltitude != null ? '${_gpsAltitude!.toStringAsFixed(1)} m' : 'N/A';
    final String pressureLabel = _pressureHPa != null ? '${_pressureHPa!.toStringAsFixed(1)} hPa' : 'N/A';
    final String speedLabel = '${_gpsSpeedMps.toStringAsFixed(2)} m/s';
    final String headingCourse = _gpsHeadingDeg != null
        ? 'Course ${_gpsHeadingDeg!.toStringAsFixed(0)}°'
        : 'Course pending';
    final String? baroSubtitle = _barometricAltitude != null
        ? 'Baro ${_barometricAltitude!.toStringAsFixed(1)} m'
        : null;
    final String distanceTraveled = '${_odometerMeters.toStringAsFixed(1)} m';

    return Padding(
      padding: const EdgeInsets.all(16),
      child: SingleChildScrollView(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Realtime motion status',
              style: Theme.of(context).textTheme.titleMedium?.copyWith(fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                Expanded(
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(12),
                    child: LinearProgressIndicator(
                      minHeight: 10,
                      value: _motionReliability.clamp(0.0, 1.0),
                      backgroundColor: Colors.white12,
                      color: reliabilityColor,
                    ),
                  ),
                ),
                const SizedBox(width: 12),
                Text('$reliabilityPercent%', style: const TextStyle(fontWeight: FontWeight.bold)),
              ],
            ),
            const SizedBox(height: 16),
            Wrap(
              spacing: 12,
              runSpacing: 12,
              children: [
                _MetricCard(icon: Icons.map, label: 'Distance', value: distanceTraveled, subtitle: 'Sensor-fused path'),
                _MetricCard(
                  icon: Icons.directions_walk,
                  label: 'Session steps',
                  value: '$_relativeSteps',
                  subtitle: 'Device total $_steps',
                ),
                _MetricCard(
                  icon: Icons.directions_run,
                  label: 'Activity',
                  value: activityName,
                  subtitle: 'Confidence $confStr',
                ),
                _MetricCard(
                  icon: Icons.explore,
                  label: 'Heading',
                  value: '$headingDeg°',
                  subtitle: 'rad ${_headingRad.toStringAsFixed(2)}',
                ),
                _MetricCard(
                  icon: Icons.speed,
                  label: 'Velocity',
                  value: '$velocityMeters m/s',
                  subtitle: '|a| $accelMagnitude m/s²',
                ),
                _MetricCard(
                  icon: Icons.straighten,
                  label: 'Step length',
                  value: '$stepLength m',
                  subtitle: 'Adaptive via motion',
                ),
                _MetricCard(
                  icon: Icons.my_location,
                  label: 'GPS accuracy',
                  value: gpsAccuracyLabel,
                  subtitle: _geoStatusMessage,
                ),
                _MetricCard(
                  icon: Icons.height,
                  label: 'Altitude',
                  value: altitudeLabel,
                  subtitle: baroSubtitle ?? 'Awaiting barometer',
                ),
                _MetricCard(icon: Icons.av_timer, label: 'Ground speed', value: speedLabel, subtitle: headingCourse),
                _MetricCard(
                  icon: Icons.radar,
                  label: 'Pressure',
                  value: pressureLabel,
                  subtitle: baroSubtitle ?? 'Sea level $_seaLevelPressureHPa hPa',
                ),
              ],
            ),
            const SizedBox(height: 20),
            _buildNavigatorPanel(),
            const SizedBox(height: 20),
            _buildLocationDiagnostics(),
            const SizedBox(height: 20),
            Container(
              width: double.infinity,
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(18),
                color: Colors.white.withValues(alpha: 0.03),
                border: Border.all(color: Colors.white10),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text('Raw sensor streams', style: TextStyle(fontWeight: FontWeight.bold, fontSize: 15)),
                  const SizedBox(height: 12),
                  _buildSensorLine('Accelerometer', accelText),
                  _buildSensorLine('Gyroscope', gyroText),
                  _buildSensorLine('Magnetometer', magText),
                  const SizedBox(height: 12),
                  Text('Relative position: x=${_position.dx.toStringAsFixed(1)}, y=${_position.dy.toStringAsFixed(1)}'),
                  Text(
                    'Relative velocity: vx=${_velocity.dx.toStringAsFixed(2)}, vy=${_velocity.dy.toStringAsFixed(2)}',
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSensorLine(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          SizedBox(
            width: 120,
            child: Text(label, style: const TextStyle(fontWeight: FontWeight.w600)),
          ),
          Expanded(child: Text(value)),
        ],
      ),
    );
  }

  Widget _buildNavigatorPanel() {
    final List<_MallNode> destinations = _mallGraph.destinationsForFloor(_selectedFloor);
    final bool hasDestinations = destinations.isNotEmpty;
    final _MallNode? currentNode = _currentNode;
    final String currentZoneLabel = currentNode != null
        ? 'You are near: ${currentNode.name} · Floor ${currentNode.floor}'
        : 'Calibrating position · Floor $_selectedFloor';
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(18),
        color: Colors.white.withValues(alpha: 0.03),
        border: Border.all(color: Colors.white10),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              const Icon(Icons.route, color: Colors.lightBlueAccent),
              const SizedBox(width: 8),
              const Text('Mall navigator', style: TextStyle(fontWeight: FontWeight.bold, fontSize: 16)),
              const Spacer(),
              DropdownButton<String>(
                value: hasDestinations ? _targetNodeId : null,
                hint: const Text('No destinations'),
                dropdownColor: Colors.grey.shade900,
                items: destinations.map((node) => DropdownMenuItem(value: node.id, child: Text(node.name))).toList(),
                onChanged: hasDestinations
                    ? (value) {
                        if (value == null) return;
                        setState(() {
                          _targetNodeId = value;
                          _updateNavigationSolution();
                        });
                      }
                    : null,
              ),
            ],
          ),
          const SizedBox(height: 8),
          Wrap(
            spacing: 8,
            runSpacing: 8,
            children: _mallFloors
                .map(
                  (floor) => ChoiceChip(
                    label: Text('Floor $floor'),
                    selected: _selectedFloor == floor,
                    onSelected: (_) {
                      if (_selectedFloor == floor) return;
                      setState(() {
                        _selectedFloor = floor;
                        _ensureTargetMatchesFloor();
                        _updateNavigationSolution();
                      });
                    },
                  ),
                )
                .toList(),
          ),
          const SizedBox(height: 8),
          Text(currentZoneLabel, style: const TextStyle(color: Colors.white70)),
          const SizedBox(height: 12),
          if (_navInstructions.isEmpty)
            const Text('Waiting for a stable fix to compute the path...', style: TextStyle(color: Colors.white60))
          else
            ..._navInstructions
                .take(4)
                .map(
                  (instruction) => ListTile(
                    dense: true,
                    contentPadding: EdgeInsets.zero,
                    leading: Icon(instruction.icon, color: Colors.amberAccent),
                    title: Text(instruction.text),
                    subtitle: Text('${instruction.distanceMeters.toStringAsFixed(0)} m'),
                  ),
                ),
        ],
      ),
    );
  }

  Widget _buildLocationDiagnostics() {
    final String latLon = _lastPosition != null
        ? '${_lastPosition!.latitude.toStringAsFixed(6)}, ${_lastPosition!.longitude.toStringAsFixed(6)}'
        : 'N/A';
    final String accuracy = _gpsAccuracyMeters.isFinite ? '${_gpsAccuracyMeters.toStringAsFixed(1)} m' : 'N/A';
    final String altitude = _gpsAltitude != null ? '${_gpsAltitude!.toStringAsFixed(1)} m' : 'N/A';
    final String speed = '${_gpsSpeedMps.toStringAsFixed(2)} m/s';
    final String heading = _gpsHeadingDeg != null ? '${_gpsHeadingDeg!.toStringAsFixed(0)}°' : 'N/A';
    final String pressure = _pressureHPa != null ? '${_pressureHPa!.toStringAsFixed(1)} hPa' : 'N/A';
    final String recency = _lastGpsFixTime != null
        ? '${DateTime.now().difference(_lastGpsFixTime!).inSeconds}s ago'
        : 'No fix';

    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(18),
        color: Colors.white.withValues(alpha: 0.02),
        border: Border.all(color: Colors.white10),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Text('Geolocation telemetry', style: TextStyle(fontWeight: FontWeight.bold, fontSize: 15)),
          const SizedBox(height: 12),
          _buildSensorLine('Lat / Lon', latLon),
          _buildSensorLine('Accuracy', accuracy),
          _buildSensorLine('Altitude', altitude),
          _buildSensorLine('Speed', speed),
          _buildSensorLine('Course', heading),
          _buildSensorLine('Pressure', pressure),
          _buildSensorLine('Last fix', recency),
          _buildSensorLine('Status', _geoStatusMessage),
          _buildSensorLine('AR status', _arStatus),
        ],
      ),
    );
  }

  void _handlePositionSample(Position position) {
    _lastPosition = position;
    _geoProjector.ensureReference(position.latitude, position.longitude);
    final Offset localMeters = _geoProjector.project(position.latitude, position.longitude);
    final Offset projected = localMeters * _metersToPixels;
    if (_gpsFilteredOffset == null) {
      _gpsFilteredOffset = projected;
    } else {
      _gpsFilteredOffset = _smoothOffset(_gpsFilteredOffset!, projected, 0.25);
    }
    _gpsOffset = _gpsFilteredOffset;
    _gpsAccuracyMeters = position.accuracy.isFinite ? position.accuracy : double.infinity;
    _gpsSpeedMps = position.speed.isFinite ? position.speed : 0;
    _gpsAltitude = position.altitude.isFinite ? position.altitude : null;
    _gpsHeadingDeg = position.heading.isFinite ? position.heading : null;
    _lastGpsFixTime = DateTime.now();
    _gpsTrust = _computeGpsTrust();
    _geoStatusMessage = 'GPS fix locked';
    if (_gpsOffset != null) {
      _applyGpsCorrection(_gpsOffset!, _gpsTrust);
    }
    _updateFusedPosition();
    setState(() {});
  }

  void _handleStepEvent(int absoluteSteps) {
    if (absoluteSteps < 0) return;
    _stepBaseline ??= absoluteSteps;
    final relative = max(0, absoluteSteps - _stepBaseline!);
    final delta = relative - _relativeSteps;
    if (delta > 0) {
      final Offset headingVector = Offset(cos(_headingRad), sin(_headingRad));
      final double stepLength = _dynamicStepLengthMeters();
      final double distanceMeters = stepLength * delta;
      final double distancePixels = distanceMeters * _metersToPixels;
      _stepPosition = _limitOffset(_stepPosition + headingVector * distancePixels);
    }
    _relativeSteps = relative;
    _updateFusedPosition();
  }

  double _dynamicStepLengthMeters() {
    const double base = 0.72;
    switch (_currentActivity) {
      case ActivityType.running:
        return base * 1.35;
      case ActivityType.onBicycle:
        return base * 1.6;
      case ActivityType.walking:
        return base;
      case ActivityType.inVehicle:
        return base * 2;
      case ActivityType.still:
        return base * 0.4;
      case ActivityType.tilting:
        return base * 0.5;
      case ActivityType.invalid:
        return base * 0.6;
      case ActivityType.unknown:
      case ActivityType.onFoot:
      case null:
        return base * 0.7;
    }
  }

  void _updateFusedPosition() {
    final Offset inertialBlend = Offset.lerp(_stepPosition, _inertialPosition, 0.35) ?? _stepPosition;
    final double gpsWeight = _gpsWeight();
    Offset fused = inertialBlend;
    if (_gpsOffset != null && gpsWeight > 0) {
      fused = Offset(
        inertialBlend.dx * (1 - gpsWeight) + _gpsOffset!.dx * gpsWeight,
        inertialBlend.dy * (1 - gpsWeight) + _gpsOffset!.dy * gpsWeight,
      );
    }
    _position = _limitOffset(fused);
    _pushTrailPoint(_position);
    final Offset fusedMeters = fused / _metersToPixels;
    if (_lastFusedMeters != null) {
      final double deltaMeters = (fusedMeters - _lastFusedMeters!).distance;
      if (deltaMeters.isFinite && deltaMeters < 30) {
        _odometerMeters += deltaMeters;
      }
    }
    _lastFusedMeters = fusedMeters;
    _updateNavigationSolution();
  }

  void _pushTrailPoint(Offset value) {
    if (_trail.isNotEmpty && (value - _trail.last).distance < 1) {
      return;
    }
    _trail.add(value);
    if (_trail.length > 90) {
      _trail.removeAt(0);
    }
    _trailVersion++;
  }

  Offset _applyDeadZone(Offset input, double threshold) {
    final double dx = input.dx.abs() < threshold ? 0 : input.dx;
    final double dy = input.dy.abs() < threshold ? 0 : input.dy;
    return Offset(dx, dy);
  }

  bool get _isArSupported => Platform.isAndroid || Platform.isIOS;

  Offset _smoothOffset(Offset previous, Offset target, double alpha) {
    final double clamped = alpha.clamp(0.0, 1.0);
    return Offset(previous.dx + (target.dx - previous.dx) * clamped, previous.dy + (target.dy - previous.dy) * clamped);
  }

  void _applyGpsCorrection(Offset gpsOffset, double trust) {
    if (trust <= 0) return;
    final Offset inertialBlend = Offset.lerp(_stepPosition, _inertialPosition, 0.35) ?? _stepPosition;
    final Offset delta = gpsOffset - inertialBlend;
    _inertialPosition += delta * (0.08 + 0.15 * trust);
    _stepPosition += delta * (0.05 + 0.12 * trust);
  }

  double _gpsWeight() {
    if (_gpsOffset == null) return 0;
    if (!_gpsAccuracyMeters.isFinite || _gpsAccuracyMeters > 25) {
      return 0;
    }
    final double accuracyScore = _gpsAccuracyMeters.isFinite ? (1 - (_gpsAccuracyMeters / 20)).clamp(0.0, 1.0) : 0;
    final double recencyScore = _lastGpsFixTime == null
        ? 0
        : (1 - DateTime.now().difference(_lastGpsFixTime!).inSeconds / 8).clamp(0.0, 1.0);
    return (_gpsTrust * 0.6 + accuracyScore * 0.3 + recencyScore * 0.1).clamp(0.0, 0.85);
  }

  double _computeGpsTrust() {
    final double accuracyScore = _gpsAccuracyMeters.isFinite ? (1 - (_gpsAccuracyMeters - 1) / 12).clamp(0.0, 1.0) : 0;
    final double recencyScore = _lastGpsFixTime == null
        ? 0
        : (1 - DateTime.now().difference(_lastGpsFixTime!).inMilliseconds / 4000).clamp(0.0, 1.0);
    final double motionScore = _motionReliability.clamp(0.0, 1.0);
    return (accuracyScore * 0.6 + recencyScore * 0.2 + motionScore * 0.2).clamp(0.0, 1.0);
  }

  double _pressureToAltitudeMeters(double pressureHPa) {
    final double ratio = (pressureHPa / _seaLevelPressureHPa).clamp(0.01, 1.5);
    final double exponent = pow(ratio, 0.1903).toDouble();
    return 44330 * (1 - exponent);
  }

  double _computeReliability(Offset accel) {
    final double magScore = (accel.distance / 1.2).clamp(0.0, 1.0);
    final double activityScore = ((_activityConfidence ?? 45) / 100).clamp(0.0, 1.0);
    return (magScore * 0.4 + activityScore * 0.6).clamp(0.0, 1.0);
  }

  Offset _limitOffset(Offset value, {double radius = 600}) {
    final double distance = value.distance;
    if (distance <= radius) return value;
    final double scale = radius / distance;
    return value * scale;
  }

  double _normalizeAngle(double angle) => atan2(sin(angle), cos(angle));

  double _blendAngles(double base, double target, double weight) {
    final double diff = atan2(sin(target - base), cos(target - base));
    return _normalizeAngle(base + diff * weight);
  }

  String _formatActivityName(ActivityType? type) {
    if (type == null) return 'Unknown';
    final String raw = type.name.replaceAll('_', ' ');
    if (raw.isEmpty) return 'Unknown';
    return raw[0].toUpperCase() + raw.substring(1);
  }

  double _lerpDouble(double a, double b, double t) {
    final double clampedT = t.clamp(0.0, 1.0);
    return a + (b - a) * clampedT;
  }

  void _resetPosition() {
    setState(() {
      _position = Offset.zero;
      _inertialPosition = Offset.zero;
      _stepPosition = Offset.zero;
      _velocity = Offset.zero;
      _headingRad = 0;
      _steps = 0;
      _relativeSteps = 0;
      _stepBaseline = null;
      _odometerMeters = 0;
      _lastFusedMeters = null;
      _trail.clear();
      _trailVersion++;
      _motionReliability = 0.4;
      _lastAccelMagnitude = 0;
      _lastAccelSample = null;
      _lastGyroSample = null;
      _gpsOffset = null;
      _gpsFilteredOffset = null;
      _gpsAccuracyMeters = double.infinity;
      _gpsSpeedMps = 0;
      _gpsAltitude = null;
      _gpsHeadingDeg = null;
      _lastPosition = null;
      _lastGpsFixTime = null;
      _gpsTrust = 0;
      _geoStatusMessage = 'GPS recalibrating...';
      _navInstructions = <_NavInstruction>[];
      _currentNode = null;
      _activeInstruction = null;
      _relativeInstructionBearing = null;
      _distanceToNextInstruction = null;
      _geoProjector.reset();
    });
  }
}

class _GridPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..strokeWidth = 0.5
      ..color = Colors.white.withValues(alpha: 0.1);

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

class _TrailPainter extends CustomPainter {
  const _TrailPainter({required this.points, required this.version});

  final List<Offset> points;
  final int version;

  @override
  void paint(Canvas canvas, Size size) {
    if (points.length < 2) return;
    final Offset center = Offset(size.width / 2, size.height / 2);
    final Path path = Path();
    final Offset first = center + points.first;
    path.moveTo(first.dx, first.dy);
    for (int i = 1; i < points.length; i++) {
      final Offset point = center + points[i];
      path.lineTo(point.dx, point.dy);
    }

    final Paint stroke = Paint()
      ..color = Colors.orangeAccent.withValues(alpha: 0.35)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 3
      ..strokeCap = StrokeCap.round;
    canvas.drawPath(path, stroke);

    final Paint marker = Paint()..color = Colors.orangeAccent.withValues(alpha: 0.55);
    final int tailCount = min(points.length, 10);
    for (int i = points.length - tailCount; i < points.length; i++) {
      if (i < 0) continue;
      final Offset point = center + points[i];
      canvas.drawCircle(point, 2.5, marker);
    }
  }

  @override
  bool shouldRepaint(covariant _TrailPainter oldDelegate) =>
      oldDelegate.version != version || oldDelegate.points.length != points.length;
}

class _MetricCard extends StatelessWidget {
  const _MetricCard({required this.icon, required this.label, required this.value, this.subtitle});

  final IconData icon;
  final String label;
  final String value;
  final String? subtitle;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Container(
      constraints: const BoxConstraints(minWidth: 140),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(16),
        color: Colors.white.withValues(alpha: 0.04),
        border: Border.all(color: Colors.white12),
        boxShadow: [BoxShadow(color: Colors.black.withValues(alpha: 0.2), blurRadius: 10, offset: const Offset(0, 4))],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Icon(icon, size: 24, color: Colors.white70),
          const SizedBox(height: 12),
          Text(value, style: theme.textTheme.titleMedium?.copyWith(fontWeight: FontWeight.bold)),
          const SizedBox(height: 4),
          Text(label, style: const TextStyle(color: Colors.white70, fontSize: 13)),
          if (subtitle != null) ...[
            const SizedBox(height: 4),
            Text(subtitle!, style: const TextStyle(color: Colors.white54, fontSize: 12)),
          ],
        ],
      ),
    );
  }
}

class _ExponentialMovingAverage {
  _ExponentialMovingAverage({required this.alpha}) : assert(alpha > 0 && alpha <= 1);

  final double alpha;
  double? _state;

  double next(double value) {
    _state = _state == null ? value : (alpha * value + (1 - alpha) * _state!);
    return _state!;
  }
}

class _MallGeoProjector {
  double? _referenceLat;
  double? _referenceLon;

  void ensureReference(double lat, double lon) {
    _referenceLat ??= lat;
    _referenceLon ??= lon;
  }

  Offset project(double lat, double lon) {
    ensureReference(lat, lon);
    if (_referenceLat == null || _referenceLon == null) {
      return Offset.zero;
    }
    const double earthRadius = 6378137.0;
    final double meanLat = _degToRad((lat + _referenceLat!) / 2);
    final double dLat = _degToRad(lat - _referenceLat!);
    final double dLon = _degToRad(lon - _referenceLon!);
    final double east = earthRadius * dLon * cos(meanLat);
    final double north = earthRadius * dLat;
    return Offset(east, -north); // negative north so UI up is north
  }

  void reset() {
    _referenceLat = null;
    _referenceLon = null;
  }
}

class _NavInstruction {
  const _NavInstruction({
    required this.icon,
    required this.text,
    required this.distanceMeters,
    this.relativeHeading,
    this.requiresLevelChange = false,
  });

  final IconData icon;
  final String text;
  final double distanceMeters;
  final double? relativeHeading;
  final bool requiresLevelChange;
}

class _MallNode {
  const _MallNode({required this.id, required this.name, required this.position, required this.floor});

  final String id;
  final String name;
  final Offset position;
  final int floor;
}

class _MallGraph {
  _MallGraph._(this._nodes, this._adjacency);

  static const String defaultDestinationId = 'food_court';
  static const int defaultFloor = 1;

  factory _MallGraph.preset() {
    const double u = _SensorNavigationPageState._metersToPixels;
    final nodes = <String, _MallNode>{
      'entrance': _MallNode(id: 'entrance', name: 'Main entrance', position: Offset(-6 * u, 4 * u), floor: 1),
      'atrium': _MallNode(id: 'atrium', name: 'Central atrium', position: const Offset(0, 0), floor: 1),
      'elevators': _MallNode(id: 'elevators', name: 'Elevators', position: Offset(2 * u, -1.5 * u), floor: 1),
      'food_court': _MallNode(id: 'food_court', name: 'Food court', position: Offset(5.5 * u, -2 * u), floor: 1),
      'cinema': _MallNode(id: 'cinema', name: 'Cinema', position: Offset(7 * u, -4 * u), floor: 2),
      'electronics': _MallNode(id: 'electronics', name: 'Electronics hub', position: Offset(-2 * u, -3 * u), floor: 1),
      'kids_zone': _MallNode(id: 'kids_zone', name: 'Kids zone', position: Offset(-5.5 * u, -2 * u), floor: 1),
      'parking': _MallNode(id: 'parking', name: 'Parking lobby', position: Offset(-8 * u, 3 * u), floor: 1),
      'rooftop': _MallNode(id: 'rooftop', name: 'Rooftop garden', position: Offset(6 * u, -5.5 * u), floor: 3),
    };

    final Map<String, List<_MallEdge>> adjacency = {for (final key in nodes.keys) key: <_MallEdge>[]};

    void connect(String a, String b) {
      final double weight = (nodes[a]!.position - nodes[b]!.position).distance;
      adjacency[a]!.add(_MallEdge(target: b, weight: weight));
      adjacency[b]!.add(_MallEdge(target: a, weight: weight));
    }

    connect('entrance', 'atrium');
    connect('atrium', 'elevators');
    connect('atrium', 'electronics');
    connect('atrium', 'kids_zone');
    connect('kids_zone', 'parking');
    connect('electronics', 'elevators');
    connect('elevators', 'food_court');
    connect('food_court', 'cinema');
    connect('cinema', 'rooftop');
    connect('food_court', 'entrance');

    return _MallGraph._(nodes, adjacency);
  }

  final Map<String, _MallNode> _nodes;
  final Map<String, List<_MallEdge>> _adjacency;

  List<_MallNode> get destinations {
    final list = _nodes.values.toList();
    list.sort((a, b) => a.name.compareTo(b.name));
    return list;
  }

  List<int> get floors {
    final List<int> uniqueFloors = _nodes.values.map((node) => node.floor).toSet().toList();
    uniqueFloors.sort();
    return uniqueFloors;
  }

  List<_MallNode> destinationsForFloor(int floor) {
    return destinations.where((node) => node.floor == floor).toList();
  }

  _MallNode? firstNodeOnFloor(int floor) {
    final filtered = destinationsForFloor(floor);
    if (filtered.isEmpty) return null;
    return filtered.first;
  }

  _MallNode? findNode(String id) => _nodes[id];

  _MallNode? nearestNode(Offset position, {int? floor}) {
    _MallNode? best;
    double bestDistance = double.infinity;
    for (final _MallNode node in _nodes.values) {
      if (floor != null && node.floor != floor) {
        continue;
      }
      final double distance = (node.position - position).distance;
      if (distance < bestDistance) {
        bestDistance = distance;
        best = node;
      }
    }
    return best;
  }

  List<String> shortestPath(String startId, String endId, {int? floor}) {
    bool isAllowed(String nodeId) {
      final _MallNode? node = _nodes[nodeId];
      if (node == null) return false;
      if (floor != null && node.floor != floor) return false;
      return true;
    }

    if (!isAllowed(startId) || !isAllowed(endId)) {
      return <String>[];
    }
    final Set<String> visited = <String>{};
    final Map<String, double> distance = {for (final key in _nodes.keys) key: double.infinity};
    final Map<String, String?> previous = {for (final key in _nodes.keys) key: null};
    distance[startId] = 0;

    while (visited.length < _nodes.length) {
      String? current;
      double smallest = double.infinity;
      distance.forEach((nodeId, value) {
        if (!visited.contains(nodeId) && isAllowed(nodeId) && value < smallest) {
          smallest = value;
          current = nodeId;
        }
      });
      if (current == null) break;
      if (current == endId) break;
      visited.add(current!);
      for (final edge in _adjacency[current] ?? const <_MallEdge>[]) {
        if (!isAllowed(edge.target)) continue;
        final double alt = distance[current]! + edge.weight;
        if (alt < distance[edge.target]!) {
          distance[edge.target] = alt;
          previous[edge.target] = current;
        }
      }
    }

    if (distance[endId] == double.infinity) {
      return <String>[startId];
    }

    final List<String> path = <String>[];
    String? step = endId;
    while (step != null) {
      path.insert(0, step);
      step = previous[step];
    }
    return path;
  }

  List<_NavInstruction> describePath({
    required List<String> path,
    required double metersToPixels,
    required double currentHeading,
  }) {
    if (path.length < 2) return <_NavInstruction>[];
    final List<_NavInstruction> instructions = <_NavInstruction>[];
    for (int i = 0; i < path.length - 1; i++) {
      final _MallNode? from = _nodes[path[i]];
      final _MallNode? to = _nodes[path[i + 1]];
      if (from == null || to == null) continue;
      final double distanceMeters = (from.position - to.position).distance / metersToPixels;
      IconData icon;
      String text;
      double? relativeHeading;
      final bool levelChange = from.floor != to.floor;
      if (levelChange) {
        icon = Icons.stairs;
        final String dir = to.floor > from.floor ? 'Go up' : 'Go down';
        text = '$dir to floor ${to.floor} near ${to.name}';
      } else {
        final double bearing = _bearingBetween(from.position, to.position);
        final double relative = atan2(sin(bearing - currentHeading), cos(bearing - currentHeading));
        relativeHeading = relative;
        if (relative.abs() < pi / 6) {
          icon = Icons.arrow_upward;
          text = 'Continue straight to ${to.name}';
        } else if (relative > 0) {
          icon = Icons.turn_left;
          text = 'Turn left toward ${to.name}';
        } else {
          icon = Icons.turn_right;
          text = 'Turn right toward ${to.name}';
        }
      }
      instructions.add(
        _NavInstruction(
          icon: icon,
          text: text,
          distanceMeters: distanceMeters,
          relativeHeading: relativeHeading,
          requiresLevelChange: levelChange,
        ),
      );
    }
    return instructions;
  }
}

class _MallEdge {
  const _MallEdge({required this.target, required this.weight});

  final String target;
  final double weight;
}

double _degToRad(double degrees) => degrees * pi / 180.0;

double _bearingBetween(Offset from, Offset to) {
  final double dx = to.dx - from.dx;
  final double dy = to.dy - from.dy;
  return atan2(-dy, dx);
}

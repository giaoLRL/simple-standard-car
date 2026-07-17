const telemetry = require('../../utils/telemetry');

function pwmLevel(pwm) {
  var magnitude = Math.min(1000, Math.abs(pwm));
  if (pwm < -10) return 'reverse';
  if (magnitude >= 700) return 'high';
  if (magnitude >= 350) return 'medium';
  if (magnitude >= 10) return 'low';
  return 'idle';
}

function buildLegacyWheels(debug) {
  return {
    motorCount: 4,
    wheels: [
      { label: '前左 FL', pwm: debug.pwmFL || 0, enc: debug.encFL || 0, level: pwmLevel(debug.pwmFL || 0), power: Math.round(Math.min(1000, Math.abs(debug.pwmFL || 0)) / 10) },
      { label: '后左 BL', pwm: debug.pwmBL || 0, enc: debug.encBL || 0, level: pwmLevel(debug.pwmBL || 0), power: Math.round(Math.min(1000, Math.abs(debug.pwmBL || 0)) / 10) },
      { label: '前右 FR', pwm: debug.pwmFR || 0, enc: debug.encFR || 0, level: pwmLevel(debug.pwmFR || 0), power: Math.round(Math.min(1000, Math.abs(debug.pwmFR || 0)) / 10) },
      { label: '后右 BR', pwm: debug.pwmBR || 0, enc: debug.encBR || 0, level: pwmLevel(debug.pwmBR || 0), power: Math.round(Math.min(1000, Math.abs(debug.pwmBR || 0)) / 10) }
    ],
    sensors: [
      { name: 'L2', active: debug.l2 },
      { name: 'L1', active: debug.l1 },
      { name: 'M', active: debug.mid, center: true },
      { name: 'R1', active: debug.r1 },
      { name: 'R2', active: debug.r2 }
    ]
  };
}

function buildV3Wheels(debug, uiConfig) {
  var motorCount = uiConfig.motorCount || 2;
  var hasEncoder = uiConfig.hasEncoder;
  var wheels = [];
  var lPwm = debug.leftPwm !== undefined ? debug.leftPwm : (debug.lPwm || 0);
  var rPwm = debug.rightPwm !== undefined ? debug.rightPwm : (debug.rPwm || 0);

  if (motorCount === 4) {
    wheels.push({ label: '前左 FL', pwm: debug.pwmFL || 0, enc: debug.encFL || 0, level: pwmLevel(debug.pwmFL || 0), power: Math.round(Math.min(1000, Math.abs(debug.pwmFL || 0)) / 10) });
    wheels.push({ label: '后左 BL', pwm: debug.pwmBL || 0, enc: debug.encBL || 0, level: pwmLevel(debug.pwmBL || 0), power: Math.round(Math.min(1000, Math.abs(debug.pwmBL || 0)) / 10) });
    wheels.push({ label: '前右 FR', pwm: debug.pwmFR || 0, enc: debug.encFR || 0, level: pwmLevel(debug.pwmFR || 0), power: Math.round(Math.min(1000, Math.abs(debug.pwmFR || 0)) / 10) });
    wheels.push({ label: '后右 BR', pwm: debug.pwmBR || 0, enc: debug.encBR || 0, level: pwmLevel(debug.pwmBR || 0), power: Math.round(Math.min(1000, Math.abs(debug.pwmBR || 0)) / 10) });
  } else {
    wheels.push({ label: '左轮', pwm: lPwm, enc: 0, level: pwmLevel(lPwm), power: Math.round(Math.min(1000, Math.abs(lPwm)) / 10) });
    wheels.push({ label: '右轮', pwm: rPwm, enc: 0, level: pwmLevel(rPwm), power: Math.round(Math.min(1000, Math.abs(rPwm)) / 10) });
  }

  var sensors = [];
  var sensorCount = uiConfig.sensorCount || 0;
  var digital = debug.digital !== undefined ? debug.digital : (debug.d || 0);
  for (var i = 0; i < sensorCount; i++) {
    var active = !!(digital & (1 << i));
    sensors.push({
      name: 'S' + i,
      active: active,
      center: i === Math.floor(sensorCount / 2)
    });
  }

  return { motorCount: motorCount, wheels: wheels, sensors: sensors };
}

Page({
  data: {
    connected: false, connecting: false, deviceName: '', debug: {},
    wheels: [], sensors: [], motorCount: 2,
    headingStyle: 'transform:rotate(0deg)',
    speedText: '--',
    hasGyro: false, hasEncoder: false, hasSensor: false,
    detailItems: [],
    helloReceived: false
  },

  onShow() {
    if (this.unsubscribe) return;
    this.unsubscribe = telemetry.subscribe(function (state) {
      var debug = state.debug;
      var uiConfig = telemetry.getUiConfig();
      var isV3 = telemetry.connectionMode === 'v3_adaptive';
      var wheelData;
      if (isV3) {
        wheelData = buildV3Wheels(debug, uiConfig);
      } else {
        wheelData = buildLegacyWheels(debug);
      }

      var angle = isV3 ? (debug.angle || debug.angleZ || 0) : (debug.angleZ || 0);
      var speedText = '--';
      if (isV3) {
        if (debug.speedAvg !== undefined) speedText = debug.speedAvg + ' mm/s';
        else if (debug.speed !== undefined) speedText = debug.speed + ' mm/s';
        else if (debug.speedL !== undefined) speedText = debug.speedL + '/' + debug.speedR + ' mm/s';
      } else {
        speedText = debug.speedValid ? (debug.speedAvg + ' mm/s') : '未标定';
      }

      var detailItems = [];
      if (!isV3) {
        detailItems = [
          { label: '目标 L/R', value: (debug.targetL || 0) + ' / ' + (debug.targetR || 0) },
          { label: '速度修正 L/R', value: (debug.corrL || 0) + ' / ' + (debug.corrR || 0) },
          { label: '陀螺修正', value: debug.gyroTrim || 0 },
          { label: 'DYC 力矩', value: debug.dycMoment || 0 }
        ];
      } else {
        if (debug.targetL !== undefined && debug.targetR !== undefined) {
          detailItems.push({ label: '目标 L/R', value: debug.targetL + ' / ' + debug.targetR });
        }
        if (debug.corrL !== undefined || debug.corrR !== undefined) {
          detailItems.push({ label: '修正 L/R', value: (debug.corrL || 0) + ' / ' + (debug.corrR || 0) });
        }
        if (debug.gyroTrim !== undefined) {
          detailItems.push({ label: '陀螺修正', value: debug.gyroTrim });
        }
        if (debug.err !== undefined) {
          detailItems.push({ label: '偏差', value: debug.err });
        }
      }

      this.setData({
        connected: state.connected, connecting: state.connecting, deviceName: state.deviceName,
        debug: debug, motorCount: wheelData.motorCount,
        wheels: wheelData.wheels, sensors: wheelData.sensors,
        headingStyle: 'transform:rotate(' + angle + 'deg)',
        speedText: speedText,
        hasGyro: uiConfig.hasGyro, hasEncoder: uiConfig.hasEncoder,
        hasSensor: (uiConfig.sensorCount || 0) > 0,
        detailItems: detailItems,
        helloReceived: isV3
      });
    }.bind(this));
  },

  onHide() { if (this.unsubscribe) { this.unsubscribe(); this.unsubscribe = null; } },
  onUnload() { this.onHide(); },
  connectBLE() { telemetry.connect(); }
});

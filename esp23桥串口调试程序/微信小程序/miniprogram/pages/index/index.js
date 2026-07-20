const telemetry = require('../../utils/telemetry');

function generatePanels(uiConfig) {
  const panels = [];
  const params = uiConfig.params;
  if (!params || Object.keys(params).length === 0) return panels;
  const pwmMax = uiConfig.pwmMax || 1000;

  if (params.kp !== undefined || params.ki !== undefined || params.kd !== undefined) {
    const controls = [];
    if (params.kp !== undefined) controls.push({ key: 'kp', name: 'Kp', min: 0, max: 1, step: 0.01, value: params.kp / 1000 });
    if (params.ki !== undefined) controls.push({ key: 'ki', name: 'Ki', min: 0, max: 0.1, step: 0.001, value: params.ki / 1000 });
    if (params.kd !== undefined) controls.push({ key: 'kd', name: 'Kd', min: 0, max: 0.5, step: 0.01, value: params.kd / 1000 });
    if (params.outputMax !== undefined) controls.push({ key: 'outputMax', name: '输出限幅', min: 50, max: pwmMax, step: 10, value: params.outputMax });
    if (controls.length) panels.push({ id: 'pid', name: '循迹 PID', controls });
  }

  if (params.baseSpeed !== undefined || params.turnOuter !== undefined || params.turnInner !== undefined) {
    const controls = [];
    if (params.baseSpeed !== undefined) controls.push({ key: 'baseSpeed', name: '基准速度', min: 0, max: pwmMax, step: 10, value: params.baseSpeed });
    if (params.turnOuter !== undefined) controls.push({ key: 'turnOuter', name: '转弯外轮', min: 0, max: pwmMax, step: 10, value: params.turnOuter });
    if (params.turnInner !== undefined) controls.push({ key: 'turnInner', name: '转弯内轮', min: 0, max: pwmMax, step: 10, value: params.turnInner });
    if (controls.length) panels.push({ id: 'speed', name: '速度配置', controls });
  }

  if (params.straight_kp !== undefined || params.curve_kp !== undefined || params.sharp_kp !== undefined) {
    const controls = [];
    if (params.straight_kp !== undefined) controls.push({ key: 'straight_kp', name: '直道 KP', min: 0.05, max: 0.8, step: 0.01, value: params.straight_kp });
    if (params.straight_ki !== undefined) controls.push({ key: 'straight_ki', name: '直道 KI', min: 0, max: 0.05, step: 0.001, value: params.straight_ki });
    if (params.straight_kd !== undefined) controls.push({ key: 'straight_kd', name: '直道 KD', min: 0, max: 0.5, step: 0.01, value: params.straight_kd });
    if (params.curve_kp !== undefined) controls.push({ key: 'curve_kp', name: '弯道 KP', min: 0.1, max: 1, step: 0.01, value: params.curve_kp });
    if (params.sharp_kp !== undefined) controls.push({ key: 'sharp_kp', name: '直角弯 KP', min: 0.1, max: 1.5, step: 0.01, value: params.sharp_kp });
    if (params.output_max !== undefined) controls.push({ key: 'output_max', name: '输出限幅', min: 100, max: pwmMax, step: 10, value: params.output_max });
    if (controls.length) panels.push({ id: 'path_pid', name: '路径 PID', controls });
  }

  if (params.chassis_pwm !== undefined || params.straight_pwm !== undefined || params.curve_pwm !== undefined) {
    const controls = [];
    if (params.straight_pwm !== undefined) controls.push({ key: 'straight_pwm', name: '直道 PWM', min: 0, max: pwmMax, step: 10, value: params.straight_pwm });
    if (params.curve_pwm !== undefined) controls.push({ key: 'curve_pwm', name: '弯道 PWM', min: 0, max: pwmMax, step: 10, value: params.curve_pwm });
    if (params.lost_diff !== undefined) controls.push({ key: 'lost_diff', name: '丢线差速', min: 50, max: pwmMax, step: 10, value: params.lost_diff });
    if (controls.length) panels.push({ id: 'chassis', name: '底盘配置', controls });
  }

  if (params.speedKp !== undefined || params.speedKi !== undefined || params.speedKd !== undefined) {
    const controls = [];
    if (params.speedKp !== undefined) controls.push({ key: 'speedKp', name: '速度 KP', min: 0, max: 2, step: 0.01, value: params.speedKp / 1000 });
    if (params.speedKi !== undefined) controls.push({ key: 'speedKi', name: '速度 KI', min: 0, max: 0.2, step: 0.001, value: params.speedKi / 1000 });
    if (params.speedKd !== undefined) controls.push({ key: 'speedKd', name: '速度 KD', min: 0, max: 0.5, step: 0.01, value: params.speedKd / 1000 });
    if (controls.length) panels.push({ id: 'speed_pid', name: '速度 PID', controls });
  }

  if (params.turnTimeoutMs !== undefined || params.turnAdvanceMs !== undefined) {
    const controls = [];
    if (params.turnTimeoutMs !== undefined) controls.push({ key: 'turnTimeoutMs', name: '直角弯超时(ms)', min: 200, max: 10000, step: 100, value: params.turnTimeoutMs });
    if (params.turnAdvanceMs !== undefined) controls.push({ key: 'turnAdvanceMs', name: '转弯前延时(ms)', min: 0, max: 1000, step: 10, value: params.turnAdvanceMs });
    if (controls.length) panels.push({ id: 'turn_cfg', name: '直角弯配置', controls });
  }

  return panels;
}

Page({
  data: {
    connected: false, connecting: false, deviceName: '', activePanel: '',
    debug: {}, currentMode: 'POS', stopped: false,
    panels: [], panelNames: [], changed: false,
    config: { wheelMm: 0, encoderPpr: 0 },
    hasEncoder: false, hasGyro: false, canConfig: false, canCalibrate: false,
    helloReceived: false
  },

  onShow() {
    var curMode = telemetry.connectionMode;
    var uiConfig = telemetry.getUiConfig();
    var isV3Now = curMode === 'v3_adaptive' || curMode === 'v4_binary';
    if (isV3Now && uiConfig.params && Object.keys(uiConfig.params).length) {
      var panels = generatePanels(uiConfig);
      var panelNames = panels.map(function (p) { return { id: p.id, name: p.name }; });
      var activePanel = this.data.activePanel;
      if (!activePanel && panels.length) activePanel = panels[0].id;
      this.setData({
        panels: panels, panelNames: panelNames, activePanel: activePanel,
        helloReceived: true, changed: false
      });
    }
    if (this.unsubscribe) return;
    this.lastMode = '';
    this.unsubscribe = telemetry.subscribe(function (state) {
      var debug = state.debug;
      var uiConfig = telemetry.getUiConfig();
      var curMode = telemetry.connectionMode;
      var activePanel = this.data.activePanel;

      if (curMode !== this.lastMode || !this.data.panels.length) {
        this.lastMode = curMode;
        var panels = generatePanels(uiConfig);
        var panelNames = panels.map(function (p) { return { id: p.id, name: p.name }; });
        if (!activePanel && panels.length) activePanel = panels[0].id;
        this.setData({ panels: panels, panelNames: panelNames, activePanel: activePanel, changed: false });
      }

      this.setData({
        connected: state.connected, connecting: state.connecting, deviceName: state.deviceName,
        debug: debug, currentMode: debug.mode || 'POS', stopped: debug.stopped || false,
        config: state.config,
        hasEncoder: uiConfig.hasEncoder, hasGyro: uiConfig.hasGyro,
        canConfig: uiConfig.canConfig, canCalibrate: uiConfig.canCalibrate,
        helloReceived: curMode === 'v3_adaptive' || curMode === 'v4_binary'
      });
    }.bind(this));
  },

  onHide() { if (this.unsubscribe) { this.unsubscribe(); this.unsubscribe = null; } },
  onUnload() { this.onHide(); },

  connectBLE() { telemetry.connect(); },
  selectPanel(e) { this.setData({ activePanel: e.currentTarget.dataset.panel }); },

  sendStop() {
    telemetry.sendCmd(this.data.stopped ? 'GO' : 'STOP');
    this.setData({ stopped: !this.data.stopped });
  },
  sendCalibrate() { telemetry.sendCmd('CAL'); },
  sendReset() {
    telemetry.clearParamCache();
    telemetry.sendCmd('RST');
  },

  onParamSlider(e) {
    var key = e.currentTarget.dataset.key;
    var value = e.detail.value;
    var panels = this.data.panels.slice();
    for (var pi = 0; pi < panels.length; pi++) {
      var controls = panels[pi].controls;
      for (var ci = 0; ci < controls.length; ci++) {
        if (controls[ci].key === key) {
          panels[pi].controls[ci].value = value;
          break;
        }
      }
    }
    this.setData({ panels: panels, changed: true });
  },

  onParamChanged(e) {
    var key = e.currentTarget.dataset.key;
    var value = e.detail.value;
    var panels = this.data.panels.slice();
    for (var pi = 0; pi < panels.length; pi++) {
      var controls = panels[pi].controls;
      for (var ci = 0; ci < controls.length; ci++) {
        if (controls[ci].key === key) {
          panels[pi].controls[ci].value = value;
          break;
        }
      }
    }
    this.setData({ panels: panels, changed: true });
  },

  sendAll() {
    var panels = this.data.panels;
    for (var pi = 0; pi < panels.length; pi++) {
      var controls = panels[pi].controls;
      for (var ci = 0; ci < controls.length; ci++) {
        var ctrl = controls[ci];
        telemetry.sendParam(ctrl.key, ctrl.value);
      }
    }
    this.setData({ changed: false });
    wx.showToast({ title: '参数已发送', icon: 'success', duration: 1000 });
  },

  onWheelMm(e) {
    var value = Math.max(0, Number(e.detail.value) || 0);
    telemetry.setConfig('wheelMm', value);
  },
  onEncoderPpr(e) {
    var value = Math.max(0, Number(e.detail.value) || 0);
    telemetry.setConfig('encoderPpr', value);
  }
});

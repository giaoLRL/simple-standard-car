const telemetry = require('../../utils/telemetry');
const HISTORY = 200;
const COLORS = ['#007f82', '#d64545', '#168aad', '#e09f3e', '#7656a8', '#303740', '#198754', '#c43d35'];

var LEGACY_CHARTS = {
  pos: { label: '循迹位置', min: -450, max: 450, series: [['pos', '#007f82']] },
  pwm: { label: '四轮 PWM', min: -1000, max: 1000, series: [['pwmFL', '#007f82'], ['pwmFR', '#d64545'], ['pwmBL', '#168aad'], ['pwmBR', '#e09f3e']] },
  enc: { label: '四轮编码器', min: -50, max: 50, series: [['encFL', '#007f82'], ['encFR', '#d64545'], ['encBL', '#168aad'], ['encBR', '#e09f3e']] },
  // speed chart removed - MCU sends leftRpm/rightRpm, see rpm chart
  gyro: { label: '陀螺仪', min: -180, max: 180, series: [['gyroZ', '#7656a8'], ['angleZ', '#d64545']] },
  dyc: { label: 'DYC', min: -800, max: 800, series: [['dycYaw', '#007f82'], ['dycMoment', '#e09f3e']] }
};

function buildCharts(uiConfig) {
  var tlm = uiConfig.tlmFields;
  if (!tlm || !tlm.length) return LEGACY_CHARTS;
  var charts = {};
  var sensorCount = uiConfig.sensorCount || 0;

  if (tlm.indexOf('pos') >= 0) {
    charts.pos = { label: '位置偏差', min: -450, max: 450, series: [['pos', COLORS[0]]] };
  }

  /* PWM chart removed — replaced by speed target-vs-actual charts below */

  var normFields = [];
  for (var i = 0; i < sensorCount; i++) {
    var nName = 'n' + i;
    if (tlm.indexOf(nName) >= 0) normFields.push([nName, COLORS[i % COLORS.length]]);
  }
  if (normFields.length) {
    charts.norm = { label: '传感器归一化', min: 0, max: uiConfig.adcMax || 4095, series: normFields };
  }

  var analogFields = [];
  for (var j = 0; j < sensorCount; j++) {
    var aName = 'a' + j;
    if (tlm.indexOf(aName) >= 0) analogFields.push([aName, COLORS[j % COLORS.length]]);
  }
  if (analogFields.length) {
    charts.analog = { label: '传感器模拟量', min: 0, max: uiConfig.adcMax || 4095, series: analogFields };
  }

  if (uiConfig.hasGyro && tlm.indexOf('gyroZ') >= 0) {
    var gyroSeries = [['gyroZ', COLORS[4]]];
    if (tlm.indexOf('angle') >= 0) gyroSeries.push(['angle', COLORS[5]]);
    if (tlm.indexOf('angleZ') >= 0) gyroSeries.push(['angleZ', COLORS[5]]);
    charts.gyro = { label: '陀螺仪', min: -360, max: 360, series: gyroSeries };
  }

  if (uiConfig.hasEncoder) {
    var hasLeftErr = tlm.indexOf('leftRpm') >= 0 && tlm.indexOf('leftTgtRpm') >= 0;
    var hasRightErr = tlm.indexOf('rightRpm') >= 0 && tlm.indexOf('rightTgtRpm') >= 0;
    if (hasLeftErr) {
      charts.spdErrL = { label: '左轮偏差(RPM)', min: -200, max: 200, series: [['spdErrL', '#d64545']] };
    }
    if (hasRightErr) {
      charts.spdErrR = { label: '右轮偏差(RPM)', min: -200, max: 200, series: [['spdErrR', '#007f82']] };
    }
  }

  if (tlm.indexOf('error') >= 0 || tlm.indexOf('err') >= 0) {
    var errField = tlm.indexOf('error') >= 0 ? 'error' : 'err';
    charts.err = { label: '循迹偏差', min: -4000, max: 4000, series: [[errField, COLORS[0]]] };
  }

  return charts;
}

function buildMetrics(debug, uiConfig) {
  var items = [];
  var isV3 = telemetry.connectionMode === 'v3_adaptive' || telemetry.connectionMode === 'v4_binary';
  var sensorCount = uiConfig.sensorCount || 0;
  var isBinary = telemetry.connectionMode === 'v4_binary';

  if (!isV3) {
    items = [
      { label: '协议', value: 'V' + (debug.protocol || '?') },
      { label: '序号', value: debug.seq || 0 },
      { label: '车端时钟', value: (debug.tick || 0) + ' ms' },
      { label: '状态', value: (debug.state || 0) + ' · ' + (debug.stateName || '') },
      { label: '模式', value: debug.mode || 'POS' },
      { label: 'STBY', value: debug.stby ? '高' : '低' }
    ];
    return items;
  }

  items.push({ label: '协议', value: isBinary ? 'V4 · 二进制' : 'V3 · 自适应' });
  if (debug.seq !== undefined) items.push({ label: '序号', value: debug.seq });
  if (debug.tick !== undefined) items.push({ label: '车端时钟', value: debug.tick + ' ms' });
  if (debug.state !== undefined) items.push({ label: '状态', value: debug.state + ' · ' + (debug.stateName || '') });
  if (debug.mode !== undefined) items.push({ label: '模式', value: debug.mode });
  if (debug.err !== undefined) items.push({ label: '偏差', value: debug.err });
  if (debug.pos !== undefined) items.push({ label: '位置', value: debug.pos });

  /* v4_binary 额外指标 */
  if (isBinary) {
    if (debug.leftPwm !== undefined) items.push({ label: '左 PWM', value: debug.leftPwm });
    if (debug.rightPwm !== undefined) items.push({ label: '右 PWM', value: debug.rightPwm });
    if (uiConfig.hasEncoder) {
      if (debug.leftRpm !== undefined) {
        var lv = debug.leftRpm;
        if (debug.leftTgtRpm !== undefined) lv += ' / ' + debug.leftTgtRpm;
        items.push({ label: '左轮(R/T)', value: lv });
      }
      if (debug.rightRpm !== undefined) {
        var rv = debug.rightRpm;
        if (debug.rightTgtRpm !== undefined) rv += ' / ' + debug.rightTgtRpm;
        items.push({ label: '右轮(R/T)', value: rv });
      }
    }
    items.push({ label: '速度环', value: debug.speedLoop ? '开' : '关' });
    items.push({ label: '方向环', value: debug.dirLoop ? '开' : '关' });
  }

  return items;
}

function buildSensors(debug, uiConfig, isV3) {
  if (!isV3) {
    return [
      { name: 'L2', active: debug.l2 },
      { name: 'L1', active: debug.l1 },
      { name: 'M', active: debug.mid, center: true },
      { name: 'R1', active: debug.r1 },
      { name: 'R2', active: debug.r2 }
    ];
  }
  var sensorCount = uiConfig.sensorCount || 0;
  var digital = debug.digital !== undefined ? debug.digital : (debug.d || 0);
  var sensors = [];
  for (var i = 0; i < sensorCount; i++) {
    sensors.push({
      name: 'S' + i,
      active: !!(digital & (1 << i)),
      center: i === Math.floor(sensorCount / 2)
    });
  }
  return sensors;
}

Page({
  data: {
    connected: false, connecting: false, deviceName: '', debug: {},
    packetRate: '0.0', validPackets: 0, invalidPackets: 0, droppedPackets: 0,
    chartKey: 'pos', chartLabel: '', legend: [], charts: {}, chartKeys: [],
    metrics: [], sensors: [],
    helloReceived: false,
    customMin: '', customMax: '', zoomLevel: 1
  },

  onLoad() {
    this.history = [];
  },
  onShow() {
    if (this.unsubscribe) return;
    this.history = telemetry.state.history.slice();
    var self = this;
    this.unsubscribe = telemetry.subscribe(function (state, reason) {
      var d = state.debug;
      var uiConfig = telemetry.getUiConfig();
      var isV3 = telemetry.connectionMode === 'v3_adaptive' || telemetry.connectionMode === 'v4_binary';
      var charts = buildCharts(uiConfig);
      var chartKeys = Object.keys(charts);
      if (!self.data.chartKey || !charts[self.data.chartKey]) {
        self.setData({ chartKey: chartKeys[0] || 'pos' });
      }
      if (reason === 'telemetry') {
      self.history = state.history.map(function(row) {
        var r = Object.assign({}, row);
        r.spdErrL = (row.leftTgtRpm !== undefined && row.leftRpm !== undefined) ? (row.leftRpm - row.leftTgtRpm) : 0;
        r.spdErrR = (row.rightTgtRpm !== undefined && row.rightRpm !== undefined) ? (row.rightRpm - row.rightTgtRpm) : 0;
        return r;
      }).slice(-HISTORY);
    }
      self.setData({
        connected: state.connected, connecting: state.connecting, deviceName: state.deviceName,
        debug: d, packetRate: state.packetRate.toFixed(1), validPackets: state.validPackets,
        invalidPackets: state.invalidPackets, droppedPackets: state.droppedPackets,
        charts: charts, chartKeys: chartKeys,
        metrics: buildMetrics(d, uiConfig),
        sensors: buildSensors(d, uiConfig, isV3),
        helloReceived: isV3 || telemetry.connectionMode === 'v4_binary'
      }, function () {
        if (self.data.chartKey && charts[self.data.chartKey]) {
          self.setData({
            chartLabel: charts[self.data.chartKey].label,
            legend: charts[self.data.chartKey].series.map(function (item) { return { name: item[0], color: item[1] }; })
          });
        }
      });
      if (reason === 'telemetry') self.drawChart();
    });
  },

  onReady() { this.initChart(); },
  onHide() { if (this.unsubscribe) { this.unsubscribe(); this.unsubscribe = null; } },
  onUnload() { this.onHide(); },
  connectBLE() { telemetry.connect(); },

  selectChart(e) {
    var key = e.currentTarget.dataset.key;
    var chart = this.data.charts[key];
    this.setData({
      chartKey: key, chartLabel: chart.label,
      legend: chart.series.map(function (item) { return { name: item[0], color: item[1] }; })
    }, function () { this.drawChart(); }.bind(this));
  },

  initChart() {
    var self = this;
    wx.createSelectorQuery().in(this).select('#telemetryChart').fields({ node: true, size: true }).exec(function (res) {
      if (!res[0]) return;
      var dpr = wx.getSystemInfoSync().pixelRatio;
      self.canvas = res[0].node;
      self.ctx = self.canvas.getContext('2d');
      self.width = res[0].width;
      self.height = res[0].height;
      self.canvas.width = self.width * dpr;
      self.canvas.height = self.height * dpr;
      self.ctx.scale(dpr, dpr);
      self.drawChart();
    }.bind(this));
  },

  zoomIn() {
    var lv = (this.data.zoomLevel || 1) * 2;
    if (lv > 32) lv = 32;
    this.setData({ zoomLevel: lv });
  },
  zoomOut() {
    var lv = (this.data.zoomLevel || 1) / 2;
    if (lv < 0.125) lv = 0.125;
    this.setData({ zoomLevel: lv });
  },
  zoomReset() {
    this.setData({ zoomLevel: 1, customMin: '', customMax: '' });
  },
  onMinInput(e) { this.setData({ customMin: e.detail.value }); },
  onMaxInput(e) { this.setData({ customMax: e.detail.value }); },

  drawChart() {
    if (!this.ctx) return;
    var ctx = this.ctx, width = this.width, height = this.height;
    var chart = this.data.charts[this.data.chartKey];
    if (!chart) return;
    /* 手动缩放：用 customMin/customMax 覆盖默认量程 */
    var cmin = parseFloat(this.data.customMin);
    var cmax = parseFloat(this.data.customMax);
    if (Number.isFinite(cmin) && Number.isFinite(cmax) && cmax > cmin) {
      chart = Object.assign({}, chart, { min: cmin, max: cmax });
    } else if (this.data.zoomLevel && this.data.zoomLevel !== 1) {
      var mid = (chart.max + chart.min) / 2;
      var half = (chart.max - chart.min) / 2 / this.data.zoomLevel;
      chart = Object.assign({}, chart, { min: mid - half, max: mid + half });
    }
    var data = this.history;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#f8f9fa';
    ctx.fillRect(0, 0, width, height);
    ctx.strokeStyle = '#dfe2e6';
    ctx.lineWidth = 1;
    for (var i = 0; i <= 4; i++) {
      var y = i * height / 4;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
    }
    var range = chart.max - chart.min;
    var zeroY = height - (0 - chart.min) / range * height;
    ctx.strokeStyle = '#aeb4bb';
    ctx.beginPath(); ctx.moveTo(0, zeroY); ctx.lineTo(width, zeroY); ctx.stroke();
    if (data.length > 1) {
      chart.series.forEach(function (series) {
        ctx.strokeStyle = series[1];
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        data.forEach(function (row, index) {
          var raw = Number(row[series[0]]);
          var value = Number.isFinite(raw) ? Math.max(chart.min, Math.min(chart.max, raw)) : 0;
          var x = index * width / (data.length - 1);
          var py = height - (value - chart.min) / range * height;
          if (index === 0) ctx.moveTo(x, py); else ctx.lineTo(x, py);
        });
        ctx.stroke();
      });
    }
    ctx.fillStyle = '#69717a';
    ctx.font = '10px sans-serif';
    ctx.fillText(String(chart.max), 4, 12);
    ctx.fillText('0', 4, Math.max(12, zeroY - 3));
    ctx.fillText(String(chart.min), 4, height - 4);
  }
});

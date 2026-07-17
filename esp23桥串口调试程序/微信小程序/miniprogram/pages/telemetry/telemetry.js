const telemetry = require('../../utils/telemetry');
const HISTORY = 200;
const COLORS = ['#007f82', '#d64545', '#168aad', '#e09f3e', '#7656a8', '#303740', '#198754', '#c43d35'];

var LEGACY_CHARTS = {
  pos: { label: '循迹位置', min: -450, max: 450, series: [['pos', '#007f82']] },
  pwm: { label: '四轮 PWM', min: -1000, max: 1000, series: [['pwmFL', '#007f82'], ['pwmFR', '#d64545'], ['pwmBL', '#168aad'], ['pwmBR', '#e09f3e']] },
  enc: { label: '四轮编码器', min: -50, max: 50, series: [['encFL', '#007f82'], ['encFR', '#d64545'], ['encBL', '#168aad'], ['encBR', '#e09f3e']] },
  speed: { label: '车速 mm/s', min: -2000, max: 2000, series: [['speedL', '#168aad'], ['speedR', '#e09f3e'], ['speedAvg', '#303740']] },
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

  var pwmL = tlm.indexOf('leftPwm') >= 0 ? 'leftPwm' : (tlm.indexOf('lPwm') >= 0 ? 'lPwm' : null);
  var pwmR = tlm.indexOf('rightPwm') >= 0 ? 'rightPwm' : (tlm.indexOf('rPwm') >= 0 ? 'rPwm' : null);
  if (pwmL && pwmR) {
    charts.pwm = { label: '电机 PWM', min: -1000, max: 1000, series: [[pwmL, COLORS[0]], [pwmR, COLORS[1]]] };
  }

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

  if (uiConfig.hasEncoder && tlm.indexOf('speedL') >= 0) {
    var speedSeries = [];
    if (tlm.indexOf('speedL') >= 0) speedSeries.push(['speedL', COLORS[0]]);
    if (tlm.indexOf('speedR') >= 0) speedSeries.push(['speedR', COLORS[1]]);
    if (tlm.indexOf('speedAvg') >= 0) speedSeries.push(['speedAvg', COLORS[2]]);
    if (speedSeries.length) {
      charts.speed = { label: '车速 mm/s', min: -2000, max: 2000, series: speedSeries };
    }
  }

  if (tlm.indexOf('err') >= 0) {
    charts.err = { label: '偏差', min: -500, max: 500, series: [['err', COLORS[0]]] };
  }

  return charts;
}

function buildMetrics(debug, uiConfig) {
  var items = [];
  var isV3 = telemetry.connectionMode === 'v3_adaptive';
  var sensorCount = uiConfig.sensorCount || 0;

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

  items.push({ label: '协议', value: 'V3 · 自适应' });
  if (debug.seq !== undefined) items.push({ label: '序号', value: debug.seq });
  if (debug.tick !== undefined) items.push({ label: '车端时钟', value: debug.tick + ' ms' });
  if (debug.state !== undefined) items.push({ label: '状态', value: debug.state + ' · ' + (debug.stateName || '') });
  if (debug.mode !== undefined) items.push({ label: '模式', value: debug.mode });
  if (debug.err !== undefined) items.push({ label: '偏差', value: debug.err });
  if (debug.pos !== undefined) items.push({ label: '位置', value: debug.pos });

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
    helloReceived: false
  },

  onLoad() {
    this.history = [];
  },
  onShow() {
    if (this.unsubscribe) return;
    this.history = telemetry.state.history.slice();
    var self = this;
    this.unsubscribe = telemetry.subscribe(function (state) {
      var d = state.debug;
      var uiConfig = telemetry.getUiConfig();
      var isV3 = telemetry.connectionMode === 'v3_adaptive';
      var charts = buildCharts(uiConfig);
      var chartKeys = Object.keys(charts);
      if (!self.data.chartKey || !charts[self.data.chartKey]) {
        self.setData({ chartKey: chartKeys[0] || 'pos' });
      }
      if (state.reason === 'telemetry') self.history = state.history.slice(-HISTORY);
      self.setData({
        connected: state.connected, connecting: state.connecting, deviceName: state.deviceName,
        debug: d, packetRate: state.packetRate.toFixed(1), validPackets: state.validPackets,
        invalidPackets: state.invalidPackets, droppedPackets: state.droppedPackets,
        charts: charts, chartKeys: chartKeys,
        metrics: buildMetrics(d, uiConfig),
        sensors: buildSensors(d, uiConfig, isV3),
        helloReceived: isV3
      }, function () {
        if (self.data.chartKey && charts[self.data.chartKey]) {
          self.setData({
            chartLabel: charts[self.data.chartKey].label,
            legend: charts[self.data.chartKey].series.map(function (item) { return { name: item[0], color: item[1] }; })
          });
        }
      });
      if (state.reason === 'telemetry') self.drawChart();
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

  drawChart() {
    if (!this.ctx) return;
    var ctx = this.ctx, width = this.width, height = this.height;
    var chart = this.data.charts[this.data.chartKey];
    if (!chart) return;
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

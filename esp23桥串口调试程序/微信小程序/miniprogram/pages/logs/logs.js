const telemetry = require('../../utils/telemetry');

function clockText(timestamp) {
  if (!timestamp) return '--:--:--';
  const date = new Date(timestamp);
  return [date.getHours(), date.getMinutes(), date.getSeconds()]
    .map(function (item) { return ('0' + item).slice(-2); }).join(':');
}

function durationText(start, end) {
  if (!start) return '00:00';
  const seconds = Math.max(0, Math.floor((end - start) / 1000));
  const minutes = Math.floor(seconds / 60);
  return ('0' + minutes).slice(-2) + ':' + ('0' + (seconds % 60)).slice(-2);
}

function pwmExtra(debug) {
  var isV3 = telemetry.connectionMode === 'v3_adaptive';
  if (isV3) {
    var l = debug.leftPwm !== undefined ? debug.leftPwm : debug.lPwm;
    var r = debug.rightPwm !== undefined ? debug.rightPwm : debug.rPwm;
    if (l !== undefined && r !== undefined) return 'PWM L:' + l + ' R:' + r;
    return '';
  }
  return 'PWM ' + (debug.pwmFL || 0) + '/' + (debug.pwmFR || 0) + '/' + (debug.pwmBL || 0) + '/' + (debug.pwmBR || 0);
}

Page({
  data: {
    connected: false, connecting: false, deviceName: '', recording: false,
    count: 0, duration: '00:00', startedAt: '--:--:--', storageText: '0 KB',
    recent: [], droppedPackets: 0, invalidPackets: 0
  },

  onShow() {
    if (this.unsubscribe) return;
    var self = this;
    this.unsubscribe = telemetry.subscribe(function (state) { self.sync(state); });
    this.timer = setInterval(function () { self.sync(telemetry.state); }, 1000);
  },
  onHide() {
    clearInterval(this.timer);
    this.timer = null;
    if (this.unsubscribe) { this.unsubscribe(); this.unsubscribe = null; }
  },
  onUnload() { this.onHide(); },
  connectBLE() { telemetry.connect(); },

  sync(state) {
    var records = state.records;
    this.setData({
      connected: state.connected, connecting: state.connecting, deviceName: state.deviceName,
      recording: state.recording, count: records.length,
      duration: durationText(state.recordStartedAt, Date.now()),
      startedAt: clockText(state.recordStartedAt),
      storageText: Math.round(state.recordBytes / 1024) + ' KB',
      recent: records.slice(-8).reverse().map(function (item) {
        return {
          key: item.time + '-' + item.seq,
          time: clockText(item.time), seq: item.seq || 0,
          stateName: item.stateName || '', mode: item.mode || 'POS',
          pos: item.pos !== undefined ? item.pos : 0,
          pwmExtra: pwmExtra(item)
        };
      }),
      droppedPackets: state.droppedPackets, invalidPackets: state.invalidPackets
    });
  },

  toggleRecording() {
    if (telemetry.state.recording) telemetry.stopRecording();
    else telemetry.startRecording();
  },

  clearRecords() {
    wx.showModal({
      title: '清除记录', content: '已保存的本地调试记录将被删除。',
      success: function (res) { if (res.confirm) telemetry.clearRecords(); }
    });
  },

  exportRecords() {
    telemetry.exportCsv(function (error, path) {
      if (error) return wx.showToast({ title: error.message || '导出失败', icon: 'none' });
      if (wx.shareFileMessage) {
        wx.shareFileMessage({
          filePath: path, fileName: path.split('/').pop(),
          fail: function () { wx.showToast({ title: '文件已生成', icon: 'none' }); }
        });
      } else {
        wx.showModal({ title: '文件已生成', content: path, showCancel: false });
      }
    });
  }
});

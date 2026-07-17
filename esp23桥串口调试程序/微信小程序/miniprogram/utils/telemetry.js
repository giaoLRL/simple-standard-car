const LOG_KEY = 'carTelemetrySessionV2';
const CONFIG_KEY = 'carControlConfigV2';
const MAX_RECORDS = 18000;
const HELLO_TIMEOUT = 3000;

function finiteNumber(value, integer) {
  const number = integer ? parseInt(value, 10) : parseFloat(value);
  return Number.isFinite(number) ? number : null;
}

function inferFieldType(name) {
  if (/^(a|analog|raw)\d*$/i.test(name)) return 'analog';
  if (/^(n|norm|normalized)\d*$/i.test(name)) return 'normalized';
  if (/^(d|digital|bits)$/i.test(name)) return 'digital';
  if (/^(l|r)?(pwm|enc|speed|target|corr|trim|moment|output)/i.test(name)) return 'int';
  if (/^(seq|tick|state|flags|caps|stby)$/i.test(name)) return 'int';
  if (/^(pos|err|angle|gyro|yaw|curv|kp|ki|kd)/i.test(name)) return 'float';
  return 'float';
}

function defaultDebugLegacy() {
  return {
    protocol: 0, seq: 0, tick: 0, receivedAt: 0, state: 0, stateName: '',
    flags: 0, caps: 0, mode: 'POS', raw: 31, electricalRaw: 31, rawHex: '0x1F',
    pos: 0, curv: 0, targetL: 0, targetR: 0,
    pwmFL: 0, pwmFR: 0, pwmBL: 0, pwmBR: 0,
    encFL: 0, encFR: 0, encBL: 0, encBR: 0, encL: 0, encR: 0,
    speedL: 0, speedR: 0, speedAvg: 0, speedValid: false,
    corrL: 0, corrR: 0, gyroTrim: 0, gyroZ: 0, angleZ: 0,
    dycYaw: 0, dycMoment: 0, adj: 0, stby: false,
    speedLoop: false, gyroLoop: false, dycLoop: false, stopped: false,
    gyroValid: false, capSpeed: false, capGyro: false, capDyc: false,
    l2: false, l1: false, mid: false, r1: false, r2: false
  };
}

function defaultDebugV3() {
  return { protocol: 3, receivedAt: 0, mode: '循迹', stateName: '', stopped: false };
}

const LEGACY_STATES = ['直道', '入弯', '弯中', '出弯', '丢线', '保留', '直角弯', '十字'];

class TelemetryService {
  constructor() {
    this.initialized = false;
    this.listeners = [];
    this.deviceId = null;
    this.serviceId = null;
    this.charWrite = null;
    this.charNotify = null;
    this.buffer = '';
    this.writeQueue = [];
    this.writing = false;
    this.throttleMap = {};
    this.lastSeq = null;
    this.lastPacketAt = 0;
    this.rateWindow = [];
    this.persistCounter = 0;
    this.hello = null;
    this.fieldMap = null;
    this.helloTimer = null;
    this.connectionMode = 'unknown';
    this.state = {
      connected: false, connecting: false, deviceName: '', packetRate: 0,
      validPackets: 0, invalidPackets: 0, droppedPackets: 0,
      recording: false, recordStartedAt: 0, records: [], recordBytes: 0, history: [],
      debug: defaultDebugLegacy(),
      config: { wheelMm: 0, encoderPpr: 0 }
    };
  }

  init() {
    if (this.initialized) return;
    this.initialized = true;
    try {
      const config = wx.getStorageSync(CONFIG_KEY);
      if (config) this.state.config = Object.assign({}, this.state.config, config);
      const session = wx.getStorageSync(LOG_KEY);
      if (session && Array.isArray(session.records)) {
        this.state.records = session.records.slice(-MAX_RECORDS);
        this.state.recordStartedAt = session.startedAt || 0;
        this.state.recordBytes = JSON.stringify(this.state.records).length;
      }
    } catch (e) {
      console.warn('本地调试记录恢复失败', e);
    }

    wx.onBLEConnectionStateChange((res) => {
      if (!res.connected && this.deviceId === res.deviceId) this.handleDisconnect();
    });
    wx.onBLECharacteristicValueChange((res) => this.onData(res.value));
  }

  subscribe(listener) {
    this.listeners.push(listener);
    listener(this.state, 'snapshot');
    return () => {
      this.listeners = this.listeners.filter((item) => item !== listener);
    };
  }

  emit(reason) {
    this.listeners.slice().forEach((listener) => listener(this.state, reason));
  }

  getUiConfig() {
    if (this.hello) {
      const caps = this.hello.caps || 0;
      return {
        deviceName: this.hello.dev || '未知设备',
        mcu: this.hello.mcu || '',
        fw: this.hello.fw || '',
        sensorCount: this.hello.sensors || 0,
        motorCount: this.hello.motors || 2,
        hasEncoder: (this.hello.encoders || 0) > 0,
        hasGyro: !!(this.hello.gyro),
        hasAnalog: !!(caps & 1),
        hasNormalized: !!(caps & 2),
        hasDigital: !!(caps & 4),
        canConfig: !!(caps & 8),
        canCalibrate: !!(caps & 16),
        states: this.hello.states || ['未知'],
        params: this.hello.params || {},
        adcMax: this.hello.adcMax || 4095,
        pwmMax: this.hello.pwmMax || 1000,
        tlmFields: this.hello.tlm || []
      };
    }
    return {
      deviceName: '', mcu: '', fw: '',
      sensorCount: 5, motorCount: 4, hasEncoder: true, hasGyro: true,
      hasAnalog: false, hasNormalized: false, hasDigital: true,
      canConfig: true, canCalibrate: false,
      states: LEGACY_STATES, params: {},
      adcMax: 4095, pwmMax: 1000, tlmFields: []
    };
  }

  connect() {
    if (this.state.connected) return this.disconnect();
    if (this.state.connecting) return;
    this.state.connecting = true;
    this.connectionMode = 'unknown';
    this.hello = null;
    this.fieldMap = null;
    if (this.helloTimer) { clearTimeout(this.helloTimer); this.helloTimer = null; }
    this.emit('connection');
    wx.closeBluetoothAdapter({ complete: () => setTimeout(() => this.openAdapter(), 250) });
  }

  openAdapter() {
    wx.openBluetoothAdapter({
      success: () => this.startDiscovery(),
      fail: () => this.connectionFailed('请打开手机蓝牙')
    });
  }

  startDiscovery() {
    const found = (res) => {
      const device = res.devices.find((item) => {
        const name = (item.name || item.localName || '').toUpperCase();
        return name === 'JDY-23' || name.startsWith('JDY-23');
      });
      if (!device) return;
      wx.offBluetoothDeviceFound(found);
      wx.stopBluetoothDevicesDiscovery();
      clearTimeout(this.scanTimer);
      this.pair(device.deviceId, device.name || device.localName || 'JDY-23', 0);
    };
    wx.onBluetoothDeviceFound(found);
    wx.startBluetoothDevicesDiscovery({
      success: () => {
        this.scanTimer = setTimeout(() => {
          wx.offBluetoothDeviceFound(found);
          wx.stopBluetoothDevicesDiscovery();
          if (!this.state.connected) this.connectionFailed('未找到 JDY-23');
        }, 10000);
      },
      fail: () => this.connectionFailed('蓝牙扫描失败')
    });
  }

  pair(deviceId, name, attempt) {
    wx.createBLEConnection({
      deviceId,
      success: () => {
        this.deviceId = deviceId;
        if (wx.setBLEMTU) wx.setBLEMTU({ deviceId, mtu: 256 });
        wx.getBLEDeviceServices({
          deviceId,
          success: (res) => {
            const service = res.services.find((item) => item.uuid.toUpperCase() === '0000FFE0-0000-1000-8000-00805F9B34FB');
            if (!service) return this.connectionFailed('设备服务不兼容');
            this.serviceId = service.uuid;
            this.findCharacteristics(deviceId, service.uuid, name);
          },
          fail: () => this.retryPair(deviceId, name, attempt, '服务发现失败')
        });
      },
      fail: () => this.retryPair(deviceId, name, attempt, '连接失败')
    });
  }

  retryPair(deviceId, name, attempt, message) {
    if (attempt < 2) {
      wx.closeBLEConnection({
        deviceId,
        complete: () => setTimeout(() => this.pair(deviceId, name, attempt + 1), 800)
      });
      return;
    }
    this.connectionFailed(message);
  }

  findCharacteristics(deviceId, serviceId, name) {
    wx.getBLEDeviceCharacteristics({
      deviceId, serviceId,
      success: (res) => {
        const target = res.characteristics.find((item) => item.uuid.toUpperCase() === '0000FFE1-0000-1000-8000-00805F9B34FB');
        if (!target || !target.properties.write) return this.connectionFailed('写特性不可用');
        this.charWrite = target.uuid;
        this.charNotify = target.uuid;
        this.enableNotify(deviceId, serviceId, target.uuid, name, 0);
      },
      fail: () => this.connectionFailed('特性发现失败')
    });
  }

  enableNotify(deviceId, serviceId, characteristicId, name, attempt) {
    wx.notifyBLECharacteristicValueChange({
      deviceId, serviceId, characteristicId, state: true,
      success: () => {
        this.state.connected = true;
        this.state.connecting = false;
        this.state.deviceName = name;
        this.buffer = '';
        this.connectionMode = 'wait_hello';
        this.helloTimer = setTimeout(() => this.onHelloTimeout(), HELLO_TIMEOUT);
        this.sendCmd('HELLO');
        this.emit('connection');
        wx.showToast({ title: '已连接', icon: 'success' });
      },
      fail: () => {
        if (attempt < 2) {
          setTimeout(() => this.enableNotify(deviceId, serviceId, characteristicId, name, attempt + 1), 500);
        } else this.connectionFailed('数据通知启用失败');
      }
    });
  }

  connectionFailed(message) {
    this.state.connecting = false;
    this.state.connected = false;
    this.connectionMode = 'unknown';
    if (this.helloTimer) { clearTimeout(this.helloTimer); this.helloTimer = null; }
    this.emit('connection');
    wx.showToast({ title: message, icon: 'none', duration: 2500 });
  }

  onHelloTimeout() {
    if (this.connectionMode === 'wait_hello') {
      this.connectionMode = 'v2_legacy';
      this.state.debug = defaultDebugLegacy();
      this.emit('hello_ready');
    }
  }

  handleDisconnect() {
    this.deviceId = null;
    this.serviceId = null;
    this.charWrite = null;
    this.charNotify = null;
    this.buffer = '';
    this.writeQueue = [];
    this.writing = false;
    this.hello = null;
    this.fieldMap = null;
    this.connectionMode = 'unknown';
    if (this.helloTimer) { clearTimeout(this.helloTimer); this.helloTimer = null; }
    this.state.connected = false;
    this.state.connecting = false;
    this.state.packetRate = 0;
    this.state.debug = this.connectionMode === 'v3_adaptive' ? defaultDebugV3() : defaultDebugLegacy();
    this.emit('connection');
  }

  disconnect() {
    clearTimeout(this.scanTimer);
    if (this.deviceId) wx.closeBLEConnection({ deviceId: this.deviceId });
    wx.closeBluetoothAdapter();
    this.handleDisconnect();
  }

  sendCmd(text) {
    if (!this.state.connected || !this.charWrite || !this.deviceId) {
      wx.showToast({ title: '请先连接小车', icon: 'none' });
      return false;
    }
    const framed = text + '\n';
    for (let offset = 0; offset < framed.length; offset += 20) {
      this.writeQueue.push(framed.slice(offset, offset + 20));
    }
    this.flushWriteQueue();
    return true;
  }

  sendThrottle(key, text) {
    const now = Date.now();
    if (this.throttleMap[key] && now - this.throttleMap[key] < 250) return;
    this.throttleMap[key] = now;
    this.sendCmd(text);
  }

  flushWriteQueue() {
    if (this.writing || !this.writeQueue.length) return;
    if (!this.state.connected || !this.deviceId || !this.charWrite) {
      this.writeQueue = [];
      return;
    }
    this.writing = true;
    const text = this.writeQueue.shift();
    const value = new Uint8Array(text.split('').map((char) => char.charCodeAt(0))).buffer;
    wx.writeBLECharacteristicValue({
      deviceId: this.deviceId,
      serviceId: this.serviceId,
      characteristicId: this.charWrite,
      value,
      fail: (err) => {
        console.warn('BLE 写入失败', err);
        if (err.errCode === 10004) this.handleDisconnect();
      },
      complete: () => {
        this.writing = false;
        setTimeout(() => this.flushWriteQueue(), 35);
      }
    });
  }

  onData(buffer) {
    const bytes = new Uint8Array(buffer);
    for (let i = 0; i < bytes.length; i++) this.buffer += String.fromCharCode(bytes[i]);
    if (this.buffer.length > 2048) this.buffer = this.buffer.slice(-1024);
    let newline;
    while ((newline = this.buffer.indexOf('\n')) >= 0) {
      const line = this.buffer.slice(0, newline).trim();
      this.buffer = this.buffer.slice(newline + 1);
      if (!line) continue;
      if (line.startsWith('$HELLO,')) {
        this.parseHello(line);
      } else if (line.startsWith('$T,2,')) {
        if (this.connectionMode === 'wait_hello') {
          this.connectionMode = 'v2_legacy';
          if (this.helloTimer) { clearTimeout(this.helloTimer); this.helloTimer = null; }
          this.emit('hello_ready');
        }
        this.parseV2(line);
      } else if (line.startsWith('$T,3,')) {
        if (this.connectionMode !== 'v3_adaptive') continue;
        this.parseV3(line);
      } else if (line.startsWith('$D,')) {
        if (this.connectionMode === 'wait_hello') {
          this.connectionMode = 'v2_legacy';
          if (this.helloTimer) { clearTimeout(this.helloTimer); this.helloTimer = null; }
          this.emit('hello_ready');
        }
        this.parseLegacy(line);
      }
    }
  }

  parseHello(line) {
    try {
      const commaIdx = line.indexOf(',', 7);
      const jsonStr = line.slice(commaIdx + 1);
      const data = JSON.parse(jsonStr);
      this.hello = data;
      this.fieldMap = {};
      if (Array.isArray(data.tlm)) {
        data.tlm.forEach(function (name, index) { this.fieldMap[name] = index; }.bind(this));
      }
      this.connectionMode = 'v3_adaptive';
      if (this.helloTimer) { clearTimeout(this.helloTimer); this.helloTimer = null; }
      this.state.debug = defaultDebugV3();
      this.state.debug.protocol = 3;
      if (this.hello.states) {
        this.state.debug.stateName = this.hello.states[0] || '';
      }
      this.emit('hello_ready');
    } catch (e) {
      console.warn('HELLO 解析失败', e);
    }
  }

  parseV3(line) {
    if (!this.fieldMap) return this.invalidPacket();
    const fields = line.split(',');
    if (fields.length < 2) return this.invalidPacket();
    const version = parseInt(fields[1], 10);
    if (version !== 3 || fields.length !== this.hello.tlm.length + 2) return this.invalidPacket();

    const debug = { protocol: 3, receivedAt: Date.now() };
    const tlmNames = this.hello.tlm;
    for (let i = 0; i < tlmNames.length; i++) {
      const name = tlmNames[i];
      const raw = fields[i + 2];
      const type = inferFieldType(name);
      if (type === 'int' || type === 'analog' || type === 'normalized') {
        const value = finiteNumber(raw, true);
        if (value === null) return this.invalidPacket();
        debug[name] = value;
      } else if (type === 'digital') {
        const value = parseInt(raw, 10);
        if (isNaN(value)) return this.invalidPacket();
        debug[name] = value;
      } else {
        const value = finiteNumber(raw, false);
        if (value === null) return this.invalidPacket();
        debug[name] = value;
      }
    }

    if (debug.state !== undefined && this.hello.states) {
      debug.stateName = this.hello.states[debug.state] || ('状态' + debug.state);
    }
    if (debug.flags !== undefined) {
      debug.stopped = !!(debug.flags & 4);
      if (debug.flags & 4) {
        debug.mode = '停车';
      } else if (debug.flags & 2) {
        debug.mode = '手动';
      } else {
        debug.mode = '循迹';
      }
    }

    this.acceptPacket(debug);
  }

  parseV2(line) {
    const p = line.split(',');
    if (p.length !== 34 || p[1] !== '2') return this.invalidPacket();
    const integerIndexes = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 33];
    const values = {};
    for (let i = 0; i < p.length; i++) {
      if (i < 2) continue;
      const value = finiteNumber(p[i], integerIndexes.indexOf(i) >= 0);
      if (value === null) return this.invalidPacket();
      values[i] = value;
    }
    const flags = values[5];
    const caps = values[6];
    const raw = values[7] & 0x1F;
    const debug = {
      protocol: 2, seq: values[2], tick: values[3], receivedAt: Date.now(),
      state: values[4], stateName: LEGACY_STATES[values[4]] || '未知', flags, caps,
      mode: (flags & 4) ? 'DYC' : ((flags & 2) ? 'GYRO' : ((flags & 1) ? 'SPEED' : 'POS')),
      raw, electricalRaw: values[8] & 0x1F,
      rawHex: '0x' + ('0' + raw.toString(16).toUpperCase()).slice(-2),
      pos: values[9], curv: values[10], targetL: values[11], targetR: values[12],
      pwmFL: values[13], pwmFR: values[14], pwmBL: values[15], pwmBR: values[16],
      encFL: values[17], encFR: values[18], encBL: values[19], encBR: values[20],
      encL: values[21], encR: values[22], speedL: values[23], speedR: values[24],
      speedAvg: values[25], corrL: values[26], corrR: values[27],
      gyroTrim: values[28], gyroZ: values[29], angleZ: values[30],
      dycYaw: values[31], dycMoment: values[32], stby: values[33] === 1,
      speedLoop: !!(flags & 1), gyroLoop: !!(flags & 2), dycLoop: !!(flags & 4),
      stopped: !!(flags & 8), gyroValid: !!(flags & 16), speedValid: !!(flags & 32),
      capSpeed: !!(caps & 1), capGyro: !!(caps & 2), capDyc: !!(caps & 4),
      l2: !(raw & 1), l1: !(raw & 2), mid: !(raw & 4), r1: !(raw & 8), r2: !(raw & 16)
    };
    debug.adj = (debug.pwmFL + debug.pwmBL - debug.pwmFR - debug.pwmBR) / 4;
    this.acceptPacket(debug);
  }

  parseLegacy(line) {
    const p = line.split(',');
    if (p.length < 10) return this.invalidPacket();
    const debug = Object.assign(defaultDebugLegacy(), {
      protocol: 1, receivedAt: Date.now(), state: finiteNumber(p[1], true) || 0,
      pos: finiteNumber(p[2], false) || 0, curv: finiteNumber(p[3], false) || 0,
      targetL: finiteNumber(p[4], true) || 0, targetR: finiteNumber(p[5], true) || 0,
      adj: finiteNumber(p[6], false) || 0, encL: finiteNumber(p[7], true) || 0,
      encR: finiteNumber(p[8], true) || 0, gyroZ: finiteNumber(p[9], false) || 0
    });
    debug.stateName = LEGACY_STATES[debug.state] || '未知';
    debug.pwmFL = debug.pwmBL = debug.targetL;
    debug.pwmFR = debug.pwmBR = debug.targetR;
    if (p.length > 10) debug.raw = finiteNumber(p[10], true) & 0x1F;
    debug.rawHex = '0x' + ('0' + debug.raw.toString(16).toUpperCase()).slice(-2);
    debug.l2 = !(debug.raw & 1); debug.l1 = !(debug.raw & 2); debug.mid = !(debug.raw & 4);
    debug.r1 = !(debug.raw & 8); debug.r2 = !(debug.raw & 16);
    this.acceptPacket(debug);
  }

  invalidPacket() {
    this.state.invalidPackets++;
    this.emit('health');
  }

  acceptPacket(debug) {
    const now = Date.now();
    if (debug.seq !== undefined && this.lastSeq !== null) {
      const gap = (debug.seq - this.lastSeq + 65536) % 65536;
      if (gap > 1 && gap < 32768) this.state.droppedPackets += gap - 1;
    }
    if (debug.seq !== undefined) this.lastSeq = debug.seq;
    this.lastPacketAt = now;
    this.rateWindow.push(now);
    while (this.rateWindow.length && now - this.rateWindow[0] > 2000) this.rateWindow.shift();
    this.state.packetRate = this.rateWindow.length > 1
      ? ((this.rateWindow.length - 1) * 1000 / (now - this.rateWindow[0])) : 0;
    this.state.validPackets++;
    this.state.debug = debug;
    this.state.history.push(Object.assign({}, debug));
    if (this.state.history.length > 200) this.state.history.shift();
    if (this.state.recording) this.appendRecord(debug);
    this.emit('telemetry');
  }

  setConfig(key, value) {
    if (key === 'wheelMm') value = value <= 0 ? 0 : Math.min(500, Math.max(1, value));
    if (key === 'encoderPpr') value = value <= 0 ? 0 : Math.min(100000, Math.max(1, value));
    this.state.config[key] = value;
    wx.setStorageSync(CONFIG_KEY, this.state.config);
    if (this.connectionMode === 'v3_adaptive' && this.hello) {
      const cmdMap = { wheelMm: 'WDM', encoderPpr: 'EPR' };
      if (cmdMap[key]) this.sendCmd(cmdMap[key] + '=' + value);
    } else {
      if (key === 'wheelMm') this.sendCmd('CFG_wheel_mm=' + value);
      if (key === 'encoderPpr') this.sendCmd('CFG_encoder_ppr=' + value);
    }
    this.emit('config');
  }

  sendParam(key, value) {
    const cmdMap = {
      kp: 'KP', ki: 'KI', kd: 'KD',
      baseSpeed: 'BSP', turnOuter: 'TOS', turnInner: 'TIS',
      lineTrackOn: 'LTO', motorOn: 'MTO',
      straight_kp: 'PK_straight_kp', straight_ki: 'PK_straight_ki', straight_kd: 'PK_straight_kd',
      curve_kp: 'PK_curve_kp', sharp_kp: 'PK_sharp_kp', output_max: 'PK_output_max',
      speed_kp: 'SK_kp', speed_ki: 'SK_ki', speed_imax: 'SK_imax', speed_corr_max: 'SK_corr_max',
      gyro_kp: 'GK_kp', gyro_ki: 'GK_ki', gyro_kd: 'GK_kd',
      gyro_trim_max: 'GK_trim_max', gyro_deadzone: 'GK_deadzone', gyro_ratelimit: 'GK_ratelimit',
      straight_pwm: 'CFG_straight', curve_pwm: 'CFG_curve', lost_diff: 'CFG_lost_diff'
    };
    const prefix = cmdMap[key];
    if (prefix) {
      this.sendCmd(prefix + '=' + value);
    } else {
      this.sendCmd(key + '=' + value);
    }
  }

  startRecording() {
    this.state.records = [];
    this.state.recordBytes = 0;
    this.state.recordStartedAt = Date.now();
    this.state.recording = true;
    this.persistRecords();
    this.emit('recording');
  }

  stopRecording() {
    this.state.recording = false;
    this.persistRecords();
    this.emit('recording');
  }

  clearRecords() {
    this.state.records = [];
    this.state.recordBytes = 0;
    this.state.recordStartedAt = 0;
    this.state.recording = false;
    wx.removeStorageSync(LOG_KEY);
    this.emit('recording');
  }

  appendRecord(debug) {
    const record = Object.assign({ time: Date.now() }, debug);
    this.state.records.push(record);
    this.state.recordBytes += JSON.stringify(record).length + 1;
    if (this.state.records.length > MAX_RECORDS) {
      const removed = this.state.records.shift();
      this.state.recordBytes = Math.max(0, this.state.recordBytes - JSON.stringify(removed).length - 1);
    }
    if (++this.persistCounter >= 25) {
      this.persistCounter = 0;
      this.persistRecords();
    }
  }

  persistRecords() {
    const payload = { startedAt: this.state.recordStartedAt, records: this.state.records };
    wx.setStorage({ key: LOG_KEY, data: payload });
  }

  exportCsv(done) {
    const records = this.state.records;
    if (!records.length) return done(new Error('暂无调试记录'));

    let columns;
    if (this.hello && this.hello.tlm) {
      columns = ['time'].concat(this.hello.tlm);
    } else {
      columns = [
        'time', 'tick', 'seq', 'state', 'mode', 'raw', 'pos', 'curv', 'targetL', 'targetR',
        'pwmFL', 'pwmFR', 'pwmBL', 'pwmBR', 'encFL', 'encFR', 'encBL', 'encBR', 'encL', 'encR',
        'speedL', 'speedR', 'speedAvg', 'corrL', 'corrR', 'gyroTrim', 'gyroZ', 'angleZ',
        'dycYaw', 'dycMoment', 'flags', 'caps'
      ];
    }

    const lines = [columns.join(',')];
    records.forEach((record) => {
      lines.push(columns.map((key) => record[key] === undefined ? '' : record[key]).join(','));
    });
    const name = 'car-debug-' + new Date().toISOString().replace(/[:.]/g, '-') + '.csv';
    const path = wx.env.USER_DATA_PATH + '/' + name;
    wx.getFileSystemManager().writeFile({
      filePath: path, data: '\uFEFF' + lines.join('\r\n'), encoding: 'utf8',
      success: () => done(null, path), fail: (err) => done(err)
    });
  }
}

module.exports = new TelemetryService();

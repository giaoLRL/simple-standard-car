const telemetry = require('./utils/telemetry');

App({
  telemetry,
  onLaunch() {
    telemetry.init();
  }
});

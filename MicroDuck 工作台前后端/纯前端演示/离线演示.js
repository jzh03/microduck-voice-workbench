'use strict';

const demoJoints = [
  ['20', '左髋旋转', 0], ['21', '左髋侧摆', -5], ['22', '左髋俯仰', -28],
  ['23', '左膝', 0], ['24', '左踝', 26], ['30', '颈部俯仰', 20],
  ['31', '头部俯仰', 20], ['32', '头部旋转', 0], ['33', '头部侧摆', 0],
  ['34', '鸭嘴', 0], ['10', '右髋旋转', 0], ['11', '右髋侧摆', 5],
  ['12', '右髋俯仰', 28], ['13', '右膝', 0], ['14', '右踝', -26],
];
let demoMode = null;
let demoStart = 0;
let demoSpeed = .05;
let demoUnlocked = false;
let demoMotion = 0;
let demoSample = null;
let demoSaved = false;
const demoOffsets = {shift: 0, hip: 0, knee: 0, ankle: 0};
const demoEvents = [];

function demoStatus() {
  const elapsed = (performance.now() - demoStart) / 1000;
  const running = demoMode !== null;
  const preparing = running && elapsed < 1.2;
  const manual = demoMode === 'static';
  const balancing = running && !preparing && !manual;
  const tilt = running ? Math.max(.1, (preparing && demoMode === 'standup' ? 72 * (1 - elapsed / 1.2) : 1.1) +
    .45 * Math.sin(elapsed * 3) + Math.abs(demoMotion) * 2) : null;
  return {
    simulation: true,
    title: preparing ? '模拟 · 正在准备姿态' : manual ? '模拟 · 固定站姿' : balancing ? '模拟 · 动态平衡' : '模拟 · 待机',
    tone: preparing ? 'warning' : running ? 'good' : 'neutral',
    status: preparing ? 'preparing' : manual ? 'holding' : balancing ? 'balancing' : 'off',
    phase: preparing ? 'preparing' : manual ? 'holding' : balancing ? 'balancing' : 'idle',
    message: demoMotion ? (demoMotion > 0 ? '模拟前进' : '模拟后退') + ' · ' + demoSpeed.toFixed(2) + ' m/s' : '当前为离线演示，读数为模拟数据。',
    running, healthy: balancing || (manual && !preparing), unknown: false,
    policy: preparing ? (demoMode === 'standup' ? 'stand' : 'held') : balancing ? 'walk' : null,
    can_start: !running, torque_off_verified: !running, can_confirm_balance: balancing,
    drive_unlocked: demoUnlocked, can_drive: balancing && demoUnlocked,
    can_adjust_pose: manual && !preparing, step_speed: demoSpeed,
    settling: preparing && demoMode === 'dynamic',
    tilt_deg: tilt, pitch_deg: tilt, roll_deg: running ? -.3 : null,
    volts_min: running ? 6.5 + .05 * Math.cos(elapsed) : 6.6,
    control_hz: running ? 50 : null, temperature_max: 36,
    server_time: Date.now() / 1000,
    imu_sample: demoSample, imu_busy: false,
    body_shift_mm: demoOffsets.shift, group_offsets_deg: {...demoOffsets},
    saved_path: demoSaved ? 'demo' : null,
    joints: demoJoints.map(([id, name, neutral]) => {
      const axis = {'12': 'hip', '22': 'hip', '13': 'knee', '23': 'knee', '14': 'ankle', '24': 'ankle'}[id];
      const sign = id.startsWith('2') ? -1 : 1;
      const target = neutral + (axis ? sign * demoOffsets[axis] : 0);
      const stride = axis && demoMotion ? 6 * Math.sin(elapsed * 8) * demoMotion * sign : 0;
      return {id, name, actual: target + stride, target: running ? target : null, current_ma: running ? 35 : null};
    }),
  };
}

async function demoRequest(action, body = {}) {
  const state = demoStatus();
  if (action === 'status') return state;
  switch (action) {
    case 'dynamic':
    case 'static':
    case 'standup':
      if (!state.can_start || body.supported !== true) throw new Error('请先勾选准备确认。');
      demoMode = action;
      demoStart = performance.now();
      break;
    case 'recover':
    case 'off':
      demoMode = null;
      demoUnlocked = false;
      demoMotion = 0;
      break;
    case 'halt':
      demoMotion = 0;
      break;
    case 'confirm':
      if (!state.can_confirm_balance || body.independent !== true) throw new Error('请等待模拟平衡并勾选确认。');
      demoUnlocked = true;
      break;
    case 'speed':
      demoSpeed = body.speed;
      demoMotion = 0;
      break;
    case 'drive':
      if (!state.can_drive) throw new Error('请先解锁前后移动。');
      demoMotion = body.vx;
      break;
    case 'imu':
      demoSample = {pitch_deg: .08, roll_deg: -.04, tilt_deg: .09, samples: 30, unix_time: Date.now() / 1000};
      break;
    case 'adjust':
      demoOffsets[body.kind] += body.delta;
      break;
    case 'reset':
    case 'capture':
      Object.keys(demoOffsets).forEach(key => { demoOffsets[key] = 0; });
      break;
    case 'save':
      demoSaved = true;
      break;
    default:
      throw new Error('演示中没有这个操作。');
  }
  if (action !== 'drive' && (action !== 'halt' || state.running)) {
    demoEvents.push({time: new Date().toISOString(), action});
  }
  return demoStatus();
}

function downloadDemo(event) {
  event.preventDefault();
  const report = {simulation: true, events: demoEvents, state: demoStatus()};
  const url = URL.createObjectURL(new Blob([JSON.stringify(report, null, 2)], {type: 'application/json'}));
  const link = document.createElement('a');
  link.href = url;
  link.download = 'microduck-demo-session.json';
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const { spawnSync } = require('node:child_process');
const test = require('node:test');
const ts = require('typescript');
const source = fs.readFileSync(path.join(__dirname, '../src/mocapSetup.ts'), 'utf8');
const exportsObject = {};
vm.runInNewContext(ts.transpileModule(source, {
  compilerOptions: { module: ts.ModuleKind.CommonJS },
}).outputText, { exports: exportsObject });
const { mocapSetupCommand } = exportsObject;
const config = {
  laptop_wifi_ip: '10.42.20.1', hdu_wifi_ip: '10.42.20.10', mdu_internal_ip: '10.42.10.12',
  motive_ip: '', vrpn_ip: '192.168.100.99', table_side: 'P2', gripper_profile: 'new_20261015',
  mocap_source: 'vrpn', vrpn_port: '3883', vrpn_units: 'mm', pants_profile: 'v4',
};

function capture(command) {
  // Shadow only the generated bash invocation; never run the lifecycle helper.
  const result = spawnSync('bash', ['-c', 'bash() { printf "%s\\n" "$@"; };\n' + command], { encoding: 'utf8' });
  assert.equal(result.status, 0, result.stderr);
  return result.stdout.trim().split('\n');
}

test('VRPN setup carries the confirmed fields through the real shell argument boundary', () => {
  const args = capture(mocapSetupCommand(config, 'model21800_20261015T010203Z'));
  assert.deepEqual(args, ['foxglove/helpers/hope-lifecycle', 'start-laptop',
    'model21800_20261015T010203Z', '10.42.20.1', '10.42.20.10', '10.42.10.12', 'NONE',
    'P2', 'OPTITRACK', 'DEFAULT', 'new_20261015', 'vrpn', '192.168.100.99', '3883', 'mm', 'v4']);
});

test('Motive and none select distinct setups; standalone session is generated only in the shell', () => {
  const motive = { ...config, mocap_source: 'motive', motive_ip: '169.254.10.221', vrpn_ip: '' };
  const args = capture(mocapSetupCommand(motive));
  assert.match(args[2], /^model21800_\d{8}T\d{6}Z$/);
  assert.equal(args[6], '169.254.10.221');
  assert.equal(args[11], 'motive');
  assert.equal(args[12], 'NONE');
  assert.equal(mocapSetupCommand({ ...config, mocap_source: 'none' }), '');
});

test('unexpected field content stays a literal argument and cannot execute shell text', () => {
  const value = "literal'$(printf EXECUTED);ignored";
  const args = capture(mocapSetupCommand({ ...config, vrpn_ip: value }));
  assert.equal(args[12], value);
});

test('Mac command retains capture settings and points to the configured operator container', () => {
  const command = mocapSetupCommand({ ...config, operator_profile: 'mac' });
  assert.match(command, /configured Mac operator container/);
  assert.equal(capture(command)[11], 'vrpn');
});

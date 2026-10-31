const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const ts = require('typescript');

// Shared module identity matters for LocalizedError and React context.
const cache = new Map();
function loadSource(name, extra = '') {
  if (name.endsWith('.css')) return {};
  const file = ['.ts', '.tsx'].map(ext => path.join(__dirname, '../src', name + ext)).find(fs.existsSync);
  if (!file) return require(name);
  if (cache.has(file)) return cache.get(file);
  const exports = {};
  cache.set(file, exports);
  const source = fs.readFileSync(file, 'utf8') + extra;
  vm.runInNewContext(ts.transpileModule(source, {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020, jsx: ts.JsxEmit.ReactJSX },
  }).outputText, {
    exports, ArrayBuffer, DataView, Error, encodeURIComponent,
    require: name => name.endsWith('.css') ? {} : name.startsWith('./') ? loadSource(name.slice(2)) : require(name),
  }, { filename: file });
  return exports;
}
module.exports = loadSource;

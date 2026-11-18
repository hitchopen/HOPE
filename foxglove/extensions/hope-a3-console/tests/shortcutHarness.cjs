const { bindRunnerShortcuts } = require('./loadSource.cjs')('runnerShortcuts');

// Native event plumbing with buttons supplied by each test (including the
// actual Console render/callbacks in operatorProfile.test.cjs).
module.exports = function shortcutHarness(getButton = () => undefined, hasDialog = () => false, sharedDoc) {
  const doc = sharedDoc ?? new EventTarget();
  doc.defaultView ??= new EventTarget();
  doc.hidden = false;
  doc.querySelectorAll = () => hasDialog() ? [{getClientRects:()=>[{}]}] : [];
  const panel = new EventTarget();
  panel.ownerDocument = doc;
  panel.isConnected = true;
  panel.getClientRects = () => [{}];
  panel.querySelectorAll = selector => {
    const result = getButton(selector.match(/data-runner-shortcut="(\w+)"/)?.[1]);
    return result == null ? [] : Array.isArray(result) ? result : [result];
  };
  const remove = bindRunnerShortcuts(panel);
  const event = (target, type, key, options = {}) => {
    const e = new Event(type, {cancelable:true});
    const code = /^[irs]$/i.test(key) ? `Key${key.toUpperCase()}` : key;
    Object.assign(e, {key, code, repeat:false, isComposing:false, keyCode:0,
      ctrlKey:false, metaKey:false, altKey:false, ...options});
    e.composedPath = () => options.path ?? [];
    target.dispatchEvent(e);
    return e;
  };
  return {
    panel, doc, remove,
    down: (key, options) => event(doc, 'keydown', key, options),
    up: key => event(doc, 'keyup', key),
    select: () => event(doc, 'pointerdown', '', {path:[panel,doc]}),
    reset: kind => {
      const [target, type] = kind === 'focusout' ? [doc, 'focusin']
        : kind === 'blur' ? [doc.defaultView, kind] : [doc, 'visibilitychange'];
      target.dispatchEvent(new Event(type));
    },
  };
};

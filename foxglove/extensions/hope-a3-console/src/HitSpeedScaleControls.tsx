import { ReactElement, useEffect, useState } from "react";

import { HitSpeedScaleState, parseHitSpeedScale } from "./hitSpeedScale";

export function HitSpeedScaleControls({ state, available, kernelMode, onApply }: {
  state?: HitSpeedScaleState; available: boolean; kernelMode: boolean;
  onApply: (scale: number) => Promise<void>;
}): ReactElement {
  const [draft, setDraft] = useState("1");
  const [edited, setEdited] = useState(false);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState("");
  useEffect(() => {
    if (!edited && state != undefined) { setDraft(String(state.scale)); }
  }, [state, edited]);
  const scale = parseHitSpeedScale(draft);
  const apply = async () => {
    if (scale == undefined) { return; }
    setPending(true); setError("");
    try { await onApply(scale); setEdited(false); }
    catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)); }
    finally { setPending(false); }
  };
  return <div className="plane-mode-controls hit-speed-scale-controls">
    <label>击球力度 scale
      <input aria-label="击球力度 scale" type="number" min="0" step="any"
        value={draft} disabled={!available || pending}
        onChange={(event) => { setDraft(event.target.value); setEdited(true); setError(""); }} />
    </label>
    <button type="button" disabled={!available || pending || scale == undefined || state == undefined || scale === state.scale}
      onClick={() => void apply()}>应用力度</button>
    <div>初始默认 1；目标速度 × scale。应用后从后续新目标生效。{state?.persistent ? "已启用保存，重启后自动恢复。" : "当前机器人尚未启用力度保存。"}</div>
    <div>{kernelMode ? "Kernel 模式不使用 Planner，此选项不可用。" : state == undefined
      ? "等待 Planner 状态" : `当前设置：${state.scale} · 最近已发目标：${state.lastCommand ?? "尚未发送"}`}</div>
    {edited && scale == undefined && <div role="alert">请输入大于或等于 0 的有限数值。</div>}
    {pending && <div role="status">等待 Planner 确认…</div>}
    {error && <div role="alert">{error}</div>}
  </div>;
}
